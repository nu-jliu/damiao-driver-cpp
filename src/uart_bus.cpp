#include "damiao/uart_bus.hpp"

#include <cstring>
#include <stdexcept>

#include <boost/asio.hpp>

namespace dm {

UartBus::UartBus(const std::string & device, const unsigned int baud_rate)
    : io_(), port_(io_, device) {
  using boost::asio::serial_port_base;
  port_.set_option(serial_port_base::baud_rate(baud_rate));
  port_.set_option(serial_port_base::character_size(8));
  port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
  port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
  port_.set_option(
      serial_port_base::flow_control(serial_port_base::flow_control::none));
}

UartBus::~UartBus() {
  if (port_.is_open()) {
    boost::system::error_code ec;
    port_.close(ec);
  }
}

void UartBus::send(const CanFrame & frame) {
  // UART packet format: [0xAA, 0x55, id_high, id_low, len, data..., checksum]
  std::vector<uint8_t> packet;
  packet.reserve(5 + frame.len + 1);

  packet.push_back(0xAA);  // Header byte 1
  packet.push_back(0x55);  // Header byte 2
  packet.push_back(static_cast<uint8_t>((frame.id >> 8) & 0xFF));
  packet.push_back(static_cast<uint8_t>(frame.id & 0xFF));
  packet.push_back(frame.len);

  for (uint8_t i = 0; i < frame.len; ++i) {
    packet.push_back(frame.data[i]);
  }

  // Checksum: sum of all bytes (excluding header) mod 256
  uint8_t checksum = 0;
  for (size_t i = 2; i < packet.size(); ++i) {
    checksum += packet[i];
  }
  packet.push_back(checksum);

  send_raw_(packet);
}

bool UartBus::receive(CanFrame & frame, const int timeout_ms) {
  // Expect: [0xAA, 0x55, id_high, id_low, len, data..., checksum]
  // Min packet size: header(2) + id(2) + len(1) + checksum(1) = 6, plus data
  constexpr size_t HEADER_SIZE = 5;
  constexpr size_t MAX_PACKET = HEADER_SIZE + 8 + 1;  // max 8 data bytes + checksum

  auto raw = receive_raw_(MAX_PACKET, timeout_ms);
  if (raw.size() < HEADER_SIZE + 1) {
    return false;
  }

  if (raw[0] != 0xAA || raw[1] != 0x55) {
    return false;
  }

  frame.id = (static_cast<uint32_t>(raw[2]) << 8) | raw[3];
  frame.len = raw[4];

  if (raw.size() < HEADER_SIZE + frame.len + 1) {
    return false;
  }

  std::memcpy(frame.data.data(), raw.data() + HEADER_SIZE, frame.len);

  // Verify checksum
  uint8_t checksum = 0;
  for (size_t i = 2; i < HEADER_SIZE + frame.len; ++i) {
    checksum += raw[i];
  }
  if (checksum != raw[HEADER_SIZE + frame.len]) {
    return false;
  }

  return true;
}

void UartBus::send_raw_(const std::vector<uint8_t> & data) {
  boost::asio::write(port_, boost::asio::buffer(data));
}

std::vector<uint8_t> UartBus::receive_raw_(const size_t max_len,
                                           const int timeout_ms) {
  std::vector<uint8_t> buf(max_len);
  size_t total_read = 0;

  boost::asio::steady_timer timer(io_);
  timer.expires_after(std::chrono::milliseconds(timeout_ms));

  bool timed_out = false;
  timer.async_wait([&](const boost::system::error_code & ec) {
    if (!ec) {
      timed_out = true;
      port_.cancel();
    }
  });

  boost::system::error_code read_ec;
  port_.async_read_some(
      boost::asio::buffer(buf.data(), max_len),
      [&](const boost::system::error_code & ec, std::size_t bytes_read) {
        read_ec = ec;
        total_read = bytes_read;
        timer.cancel();
      });

  io_.restart();
  io_.run();

  if (timed_out || read_ec) {
    buf.clear();
    return buf;
  }

  buf.resize(total_read);
  return buf;
}

}  // namespace dm
