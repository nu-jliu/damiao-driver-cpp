#include "damiao/can_bus.hpp"

#include <cstring>
#include <stdexcept>
#include <utility>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace dm {

CanBus::CanBus(const std::string & interface) {
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    throw std::runtime_error("Failed to create CAN socket");
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("Failed to get interface index for " + interface);
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr),
             sizeof(addr)) < 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("Failed to bind CAN socket to " + interface);
  }
}

CanBus::~CanBus() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
  }
}

CanBus::CanBus(CanBus && other) noexcept : socket_fd_(other.socket_fd_) {
  other.socket_fd_ = -1;
}

CanBus & CanBus::operator=(CanBus && other) noexcept {
  if (this != &other) {
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
    }
    socket_fd_ = other.socket_fd_;
    other.socket_fd_ = -1;
  }
  return *this;
}

void CanBus::send(const CanFrame & frame) {
  struct can_frame cf {};
  cf.can_id = frame.id;
  cf.can_dlc = frame.len;
  std::memcpy(cf.data, frame.data.data(), frame.len);

  const auto nbytes = ::write(socket_fd_, &cf, sizeof(cf));
  if (nbytes != sizeof(cf)) {
    throw std::runtime_error("Failed to write CAN frame");
  }
}

bool CanBus::receive(CanFrame & frame, const int timeout_ms) {
  struct pollfd pfd {};
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  const int ret = ::poll(&pfd, 1, timeout_ms);
  if (ret < 0) {
    throw std::runtime_error("poll() failed on CAN socket");
  }
  if (ret == 0) {
    return false;  // Timeout
  }

  struct can_frame cf {};
  const auto nbytes = ::read(socket_fd_, &cf, sizeof(cf));
  if (nbytes != sizeof(cf)) {
    throw std::runtime_error("Failed to read CAN frame");
  }

  frame.id = cf.can_id;
  frame.len = cf.can_dlc;
  std::memcpy(frame.data.data(), cf.data, cf.can_dlc);

  return true;
}

}  // namespace dm
