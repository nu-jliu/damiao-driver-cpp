#include "damiao_driver/motor.hpp"

#include <cstring>

namespace dm
{

  DmMotor::DmMotor(CommBus &bus, const uint16_t motor_id, const MotorParams &params)
      : bus_(bus), motor_id_(motor_id), params_(params) {}

  // --- Motor lifecycle ---

  Feedback DmMotor::enable()
  {
    return send_and_receive_(encode_special_(ENABLE_CMD));
  }

  Feedback DmMotor::disable()
  {
    return send_and_receive_(encode_special_(DISABLE_CMD));
  }

  Feedback DmMotor::saveZeroPosition()
  {
    return send_and_receive_(encode_special_(SAVE_ZERO_CMD));
  }

  Feedback DmMotor::clearError()
  {
    return send_and_receive_(encode_special_(CLEAR_ERROR_CMD));
  }

  // --- Control commands ---

  Feedback DmMotor::sendMit(const float p_des, const float v_des, const float kp,
                            const float kd, const float t_ff)
  {
    return send_and_receive_(encode_mit_(p_des, v_des, kp, kd, t_ff));
  }

  Feedback DmMotor::sendPositionSpeed(const float p_des, const float v_des)
  {
    return send_and_receive_(encode_position_speed_(p_des, v_des));
  }

  Feedback DmMotor::sendSpeed(const float v_des)
  {
    return send_and_receive_(encode_speed_(v_des));
  }

  // --- Accessors ---

  uint16_t DmMotor::motorId() const { return motor_id_; }

  const Feedback &DmMotor::lastFeedback() const { return last_feedback_; }

  // --- Private: Encode ---

  CanFrame DmMotor::encode_mit_(const float p_des, const float v_des, const float kp,
                                const float kd, const float t_ff) const
  {
    CanFrame frame;
    frame.id = motor_id_;
    frame.len = 8;

    const uint16_t p_int =
        float_to_uint(p_des, -params_.p_max, params_.p_max, 16);
    const uint16_t v_int =
        float_to_uint(v_des, -params_.v_max, params_.v_max, 12);
    const uint16_t kp_int = float_to_uint(kp, 0.0f, params_.kp_max, 12);
    const uint16_t kd_int = float_to_uint(kd, 0.0f, params_.kd_max, 12);
    const uint16_t t_int =
        float_to_uint(t_ff, -params_.t_max, params_.t_max, 12);

    // Bit packing (8 bytes = 64 bits):
    // D[0]:   p_int[15:8]
    // D[1]:   p_int[7:0]
    // D[2]:   v_int[11:4]
    // D[3]:   v_int[3:0] | kp_int[11:8]
    // D[4]:   kp_int[7:0]
    // D[5]:   kd_int[11:4]
    // D[6]:   kd_int[3:0] | t_int[11:8]
    // D[7]:   t_int[7:0]
    frame.data[0] = static_cast<uint8_t>(p_int >> 8);
    frame.data[1] = static_cast<uint8_t>(p_int & 0xFF);
    frame.data[2] = static_cast<uint8_t>(v_int >> 4);
    frame.data[3] = static_cast<uint8_t>(((v_int & 0xF) << 4) | (kp_int >> 8));
    frame.data[4] = static_cast<uint8_t>(kp_int & 0xFF);
    frame.data[5] = static_cast<uint8_t>(kd_int >> 4);
    frame.data[6] = static_cast<uint8_t>(((kd_int & 0xF) << 4) | (t_int >> 8));
    frame.data[7] = static_cast<uint8_t>(t_int & 0xFF);

    return frame;
  }

  CanFrame DmMotor::encode_position_speed_(const float p_des,
                                           const float v_des) const
  {
    CanFrame frame;
    frame.id = 0x100 + motor_id_;
    frame.len = 8;

    std::memcpy(frame.data.data(), &p_des, sizeof(float));
    std::memcpy(frame.data.data() + 4, &v_des, sizeof(float));

    return frame;
  }

  CanFrame DmMotor::encode_speed_(const float v_des) const
  {
    CanFrame frame;
    frame.id = 0x200 + motor_id_;
    frame.len = 8;

    std::memcpy(frame.data.data(), &v_des, sizeof(float));
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;

    return frame;
  }

  CanFrame DmMotor::encode_special_(const std::array<uint8_t, 8> &cmd) const
  {
    CanFrame frame;
    frame.id = motor_id_;
    frame.len = 8;
    frame.data = cmd;
    return frame;
  }

  // --- Private: Decode ---

  Feedback DmMotor::decode_feedback_(const CanFrame &frame) const
  {
    Feedback fb{};

    // D[0]: motor_id (upper nibble) | error (lower nibble)
    fb.motor_id = (frame.data[0] >> 4) & 0x0F;
    fb.error = static_cast<MotorError>(frame.data[0] & 0x0F);

    // D[1..2]: position (16-bit)
    const uint16_t p_int =
        (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[2];
    fb.position = uint_to_float(p_int, -params_.p_max, params_.p_max, 16);

    // D[3] full byte + D[4] upper nibble: velocity (12-bit)
    const uint16_t v_int =
        (static_cast<uint16_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
    fb.velocity = uint_to_float(v_int, -params_.v_max, params_.v_max, 12);

    // D[4] lower nibble + D[5] full byte: torque (12-bit)
    const uint16_t t_int =
        (static_cast<uint16_t>(frame.data[4] & 0x0F) << 8) | frame.data[5];
    fb.torque = uint_to_float(t_int, -params_.t_max, params_.t_max, 12);

    // D[6]: T_MOS, D[7]: T_Rotor
    fb.t_mos = frame.data[6];
    fb.t_rotor = frame.data[7];

    return fb;
  }

  // --- Private: Send and Receive ---

  Feedback DmMotor::send_and_receive_(const CanFrame &frame)
  {
    bus_.send(frame);

    CanFrame response;
    if (bus_.receive(response))
    {
      last_feedback_ = decode_feedback_(response);
    }

    return last_feedback_;
  }

} // namespace dm
