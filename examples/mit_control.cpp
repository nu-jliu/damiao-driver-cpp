#include <cstdio>

#include "damiao_driver/damiao_driver.hpp"

int main()
{
  try
  {
    dm::CanBus bus("can0");
    dm::DmMotor motor(bus, 0x01);

    // Enable motor
    auto fb = motor.enable();
    std::printf("Enabled motor %u\n", fb.motor_id);

    // Send MIT mode command: hold position at 0 with moderate stiffness
    fb = motor.sendMit(0.0f, 0.0f, 50.0f, 1.0f, 0.0f);
    std::printf("Position: %.3f rad, Velocity: %.3f rad/s, Torque: %.3f Nm\n",
                fb.position, fb.velocity, fb.torque);
    std::printf("T_MOS: %u C, T_Rotor: %u C\n", fb.t_mos, fb.t_rotor);

    // Disable motor
    motor.disable();
    std::printf("Motor disabled\n");
  }
  catch (const std::exception &e)
  {
    std::fprintf(stderr, "Error: %s\n", e.what());
    return 1;
  }

  return 0;
}
