#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <boost/asio.hpp>

#include "damiao/comm_bus.hpp"

namespace dm {

class UartBus : public CommBus {
public:
  explicit UartBus(const std::string & device, const unsigned int baud_rate = 921600);
  ~UartBus() override;

  UartBus(const UartBus &) = delete;
  UartBus & operator=(const UartBus &) = delete;

  void send(const CanFrame & frame) override;
  bool receive(CanFrame & frame, const int timeout_ms = 10) override;

private:
  void send_raw_(const std::vector<uint8_t> & data);
  std::vector<uint8_t> receive_raw_(const size_t max_len, const int timeout_ms);

  boost::asio::io_context io_;
  boost::asio::serial_port port_;
};

}  // namespace dm
