#pragma once

#include <memory>

#include "damiao_driver/comm_bus.hpp"
#include "damiao_driver/types.hpp"

namespace dm
{

  class DmMotor
  {
  public:
    /// Construct a motor with a MotorType enum (recommended).
    /// Automatically selects the correct MotorParams for the given motor model.
    DmMotor(
        std::shared_ptr<CommBus> bus,
        const uint16_t motor_id,
        const MotorType motor_type = MotorType::DM_J4310);

    /// Construct a motor with custom MotorParams (advanced usage).
    DmMotor(
        std::shared_ptr<CommBus> bus,
        const uint16_t motor_id,
        const MotorParams &params);

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
    uint16_t motorId() const;
    const Feedback &lastFeedback() const;

  private:
    CanFrame encode_mit_(const float p_des, const float v_des, const float kp,
                         const float kd, const float t_ff) const;
    CanFrame encode_position_speed_(const float p_des, const float v_des) const;
    CanFrame encode_speed_(const float v_des) const;
    CanFrame encode_special_(const std::array<uint8_t, 8> &cmd) const;

    Feedback decode_feedback_(const CanFrame &frame) const;
    Feedback send_and_receive_(const CanFrame &frame);

    std::shared_ptr<CommBus> bus_;
    uint16_t motor_id_;
    MotorParams params_;
    Feedback last_feedback_{};
  };

} // namespace dm
