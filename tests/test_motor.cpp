#include <cstring>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "damiao_driver/motor.hpp"

using Catch::Matchers::WithinAbs;

// Mock CommBus for testing (no real CAN/UART)
class MockBus : public dm::CommBus
{
public:
  dm::CanFrame last_sent{};
  dm::CanFrame next_response{};
  bool has_response = false;

  void send(const dm::CanFrame &frame) override { last_sent = frame; }

  bool receive(dm::CanFrame &frame, const int /*timeout_ms*/) override
  {
    if (has_response)
    {
      frame = next_response;
      return true;
    }
    return false;
  }
};

TEST_CASE("MIT mode encoding", "[motor]")
{
  MockBus bus;
  dm::DmMotor motor(bus, 0x01);

  SECTION("Zero command encodes to midpoints")
  {
    motor.sendMit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    const auto &f = bus.last_sent;

    REQUIRE(f.id == 0x01);
    REQUIRE(f.len == 8);

    // p_des=0 -> midpoint of 16-bit = 0x7FFF
    REQUIRE(f.data.at(0) == 0x7F);
    REQUIRE(f.data.at(1) == 0xFF);
  }

  SECTION("CAN ID is motor_id")
  {
    motor.sendMit(1.0f, 2.0f, 10.0f, 1.0f, 0.5f);
    REQUIRE(bus.last_sent.id == 0x01);
  }
}

TEST_CASE("Position-Speed mode encoding", "[motor]")
{
  MockBus bus;
  dm::DmMotor motor(bus, 0x01);

  SECTION("Encodes float values in little-endian")
  {
    const float p_des = 1.5f;
    const float v_des = 3.0f;

    motor.sendPositionSpeed(p_des, v_des);
    const auto &f = bus.last_sent;

    REQUIRE(f.id == 0x101);
    REQUIRE(f.len == 8);

    float p_decoded, v_decoded;
    std::memcpy(&p_decoded, f.data.data(), sizeof(float));
    std::memcpy(&v_decoded, f.data.data() + 4, sizeof(float));

    REQUIRE(p_decoded == p_des);
    REQUIRE(v_decoded == v_des);
  }
}

TEST_CASE("Speed mode encoding", "[motor]")
{
  MockBus bus;
  dm::DmMotor motor(bus, 0x02);

  SECTION("Encodes velocity and zeros padding")
  {
    const float v_des = 5.0f;

    motor.sendSpeed(v_des);
    const auto &f = bus.last_sent;

    REQUIRE(f.id == 0x202);
    REQUIRE(f.len == 8);

    float v_decoded;
    std::memcpy(&v_decoded, f.data.data(), sizeof(float));
    REQUIRE(v_decoded == v_des);

    REQUIRE(f.data.at(4) == 0x00);
    REQUIRE(f.data.at(5) == 0x00);
    REQUIRE(f.data.at(6) == 0x00);
    REQUIRE(f.data.at(7) == 0x00);
  }
}

TEST_CASE("Special commands produce correct frames", "[motor]")
{
  MockBus bus;
  dm::DmMotor motor(bus, 0x03);

  SECTION("Enable command")
  {
    motor.enable();
    REQUIRE(bus.last_sent.id == 0x03);
    REQUIRE(bus.last_sent.data == dm::ENABLE_CMD);
  }

  SECTION("Disable command")
  {
    motor.disable();
    REQUIRE(bus.last_sent.data == dm::DISABLE_CMD);
  }

  SECTION("Save zero position command")
  {
    motor.saveZeroPosition();
    REQUIRE(bus.last_sent.data == dm::SAVE_ZERO_CMD);
  }

  SECTION("Clear error command")
  {
    motor.clearError();
    REQUIRE(bus.last_sent.data == dm::CLEAR_ERROR_CMD);
  }
}

TEST_CASE("Feedback decoding from known bytes", "[motor]")
{
  MockBus bus;
  dm::DmMotor motor(bus, 0x01);

  // Prepare a known feedback frame
  dm::CanFrame response;
  response.id = 0x00;
  response.len = 8;

  // D[0]: motor_id=1 (upper nibble), error=None (lower nibble)
  response.data[0] = 0x10;

  // D[1..2]: position = midpoint (0x7FFF) -> ~0.0 rad
  response.data[1] = 0x7F;
  response.data[2] = 0xFF;

  // D[3] full + D[4] upper nibble: velocity = midpoint (0x7FF) -> ~0.0 rad/s
  response.data[3] = 0x7F;
  response.data[4] = 0xF0 | 0x07; // v[3:0]=0xF, t[11:8]=0x7

  // D[4] lower nibble + D[5]: torque = midpoint (0x7FF) -> ~0.0 Nm
  response.data[5] = 0xFF;

  // D[6]: T_MOS=30, D[7]: T_Rotor=25
  response.data[6] = 30;
  response.data[7] = 25;

  bus.next_response = response;
  bus.has_response = true;

  auto fb = motor.enable();

  REQUIRE(fb.motor_id == 1);
  REQUIRE(fb.error == dm::MotorError::None);
  REQUIRE_THAT(fb.position, WithinAbs(0.0f, 0.001f));
  REQUIRE_THAT(fb.velocity, WithinAbs(0.0f, 0.02f));
  REQUIRE_THAT(fb.torque, WithinAbs(0.0f, 0.01f));
  REQUIRE(fb.t_mos == 30);
  REQUIRE(fb.t_rotor == 25);
}

TEST_CASE("Accessor methods", "[motor]")
{
  MockBus bus;
  dm::DmMotor motor(bus, 0x05);

  REQUIRE(motor.motorId() == 0x05);

  const auto &fb = motor.lastFeedback();
  REQUIRE(fb.motor_id == 0); // Default-initialized
}

TEST_CASE("Wide CAN ID support", "[motor]")
{
  MockBus bus;

  SECTION("Motor ID at upper bound (0x5FF)")
  {
    dm::DmMotor motor(bus, 0x5FF);

    REQUIRE(motor.motorId() == 0x5FF);

    motor.sendMit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    REQUIRE(bus.last_sent.id == 0x5FF);

    motor.sendPositionSpeed(1.0f, 2.0f);
    REQUIRE(bus.last_sent.id == 0x6FF); // 0x100 + 0x5FF

    motor.sendSpeed(1.0f);
    REQUIRE(bus.last_sent.id == 0x7FF); // 0x200 + 0x5FF

    motor.enable();
    REQUIRE(bus.last_sent.id == 0x5FF);
    REQUIRE(bus.last_sent.data == dm::ENABLE_CMD);
  }

  SECTION("Motor ID beyond uint8_t range (0x3A7)")
  {
    dm::DmMotor motor(bus, 0x3A7);

    REQUIRE(motor.motorId() == 0x3A7);

    motor.sendMit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    REQUIRE(bus.last_sent.id == 0x3A7);

    motor.sendPositionSpeed(1.0f, 2.0f);
    REQUIRE(bus.last_sent.id == 0x4A7); // 0x100 + 0x3A7

    motor.sendSpeed(1.0f);
    REQUIRE(bus.last_sent.id == 0x5A7); // 0x200 + 0x3A7
  }
}
