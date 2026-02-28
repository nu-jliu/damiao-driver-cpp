#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>

namespace dm
{

  // --- Enums ---

  enum class ControlMode : uint8_t
  {
    Mit,
    PositionSpeed,
    Speed
  };

  enum class MotorError : uint8_t
  {
    None = 0x0,
    Overvoltage = 0x8,
    Undervoltage = 0x9,
    Overcurrent = 0xA,
    MosOvertemp = 0xB,
    MotorOvertemp = 0xC,
    CommLoss = 0xD,
    Overload = 0xE
  };

  /// Motor type enum for supported Damiao motor models
  enum class MotorType : uint8_t
  {
    DM_J4310, ///< DM-J4310-2EC: 10:1 gear ratio, 10Nm peak, 30 rad/s
    DM_J4340  ///< DM-J4340-2EC: 40:1 gear ratio, 28Nm peak, 10 rad/s
  };

  // --- Structs ---

  struct MotorParams
  {
    float p_max;  // Max position (rad) for MIT linear mapping
    float v_max;  // Max velocity (rad/s) for MIT linear mapping
    float t_max;  // Max torque (Nm) for MIT linear mapping
    float kp_max; // Max Kp gain
    float kd_max; // Max Kd gain
  };

  /// Default parameters for DM-J4310-2EC (gear ratio 10:1)
  constexpr MotorParams DM_J4310_DEFAULTS = {12.5f, 30.0f, 10.0f, 500.0f, 5.0f};

  /// Default parameters for DM-J4340-2EC (gear ratio 40:1)
  constexpr MotorParams DM_J4340_DEFAULTS = {12.5f, 10.0f, 28.0f, 500.0f, 5.0f};

  /// Look up default MotorParams for a given MotorType
  inline constexpr MotorParams motor_params_for(MotorType type)
  {
    switch (type)
    {
    case MotorType::DM_J4310:
      return DM_J4310_DEFAULTS;
    case MotorType::DM_J4340:
      return DM_J4340_DEFAULTS;
    default:
      return DM_J4310_DEFAULTS;
    }
  }

  struct Feedback
  {
    uint16_t motor_id;
    MotorError error;
    float position;  // rad
    float velocity;  // rad/s
    float torque;    // Nm
    uint8_t t_mos;   // MOS temperature (C)
    uint8_t t_rotor; // Rotor temperature (C)
  };

  struct CanFrame
  {
    uint32_t id;
    uint8_t len;
    std::array<uint8_t, 8> data{};
  };

  // --- Linear mapping (MIT mode encoding/decoding) ---

  uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
  float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);

  // --- Enable/Disable/Clear special command bytes ---

  constexpr std::array<uint8_t, 8> ENABLE_CMD = {
      0xFF, 0xFF, 0xFF, 0xFF,
      0xFF, 0xFF, 0xFF, 0xFC};
  constexpr std::array<uint8_t, 8> DISABLE_CMD = {
      0xFF, 0xFF, 0xFF, 0xFF,
      0xFF, 0xFF, 0xFF, 0xFD};
  constexpr std::array<uint8_t, 8> SAVE_ZERO_CMD = {
      0xFF, 0xFF, 0xFF, 0xFF,
      0xFF, 0xFF, 0xFF, 0xFE};
  constexpr std::array<uint8_t, 8> CLEAR_ERROR_CMD = {
      0xFF, 0xFF, 0xFF, 0xFF,
      0xFF, 0xFF, 0xFF, 0xFB};

} // namespace dm
