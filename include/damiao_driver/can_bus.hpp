#pragma once

#include <string>

#include "damiao_driver/comm_bus.hpp"

namespace dm
{

  class CanBus : public CommBus
  {
  public:
    explicit CanBus(const std::string &interface);
    ~CanBus() override;

    CanBus(const CanBus &) = delete;
    CanBus &operator=(const CanBus &) = delete;
    CanBus(CanBus &&other) noexcept;
    CanBus &operator=(CanBus &&other) noexcept;

    void send(const CanFrame &frame) override;
    bool receive(CanFrame &frame, const int timeout_ms = 10) override;

  private:
    int socket_fd_ = -1;
  };

} // namespace dm
