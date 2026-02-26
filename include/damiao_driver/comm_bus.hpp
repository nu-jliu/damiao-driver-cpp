#pragma once

#include "damiao_driver/types.hpp"

namespace dm
{

  class CommBus
  {
  public:
    virtual ~CommBus() = default;

    virtual void send(const CanFrame &frame) = 0;
    virtual bool receive(CanFrame &frame, const int timeout_ms = 10) = 0;
  };

} // namespace dm
