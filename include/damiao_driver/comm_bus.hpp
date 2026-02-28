#pragma once

#include "damiao_driver/types.hpp"

namespace dm
{

  class CommBus
  {
  public:
    virtual ~CommBus() = default;

    CommBus(const CommBus &) = delete;
    CommBus &operator=(const CommBus &) = delete;
    CommBus(CommBus &&) = delete;
    CommBus &operator=(CommBus &&) = delete;

    virtual void send(const CanFrame &frame) = 0;
    virtual bool receive(CanFrame &frame, const int timeout_ms = 10) = 0;

  protected:
    CommBus() = default;
  };

} // namespace dm
