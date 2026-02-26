#pragma once

#include "damiao_driver/comm_bus.hpp"
#include "damiao_driver/types.hpp"

namespace dm
{

  class DmMotor
  {
  public:
    DmMotor(
        CommBus &bus,
        const uint8_t motor_id,
        const MotorParams &params = DM_J4310_DEFAULTS);

    // --- Motor lifecycle ---
    Feedback enable();
    Feedback disable();
    Feedback saveZeroPosition();
    Feedback clearError();

    // --- Control commands (each returns motor feedback) ---
    Feedback sendMit(const float p_des, const float v_des, const float kp,
                     const float kd, const float t_ff);
    Feedback sendPositionSpeed(const float p_des, const float v_des);
    Feedback sendSpeed(const float v_des);

    // --- Accessors ---
    uint8_t motorId() const;
    const Feedback &lastFeedback() const;

  private:
    CanFrame encode_mit_(const float p_des, const float v_des, const float kp,
                         const float kd, const float t_ff) const;
    CanFrame encode_position_speed_(const float p_des, const float v_des) const;
    CanFrame encode_speed_(const float v_des) const;
    CanFrame encode_special_(const std::array<uint8_t, 8> &cmd) const;

    Feedback decode_feedback_(const CanFrame &frame) const;
    Feedback send_and_receive_(const CanFrame &frame);

    CommBus &bus_;
    uint8_t motor_id_;
    MotorParams params_;
    Feedback last_feedback_{};
  };

} // namespace dm
