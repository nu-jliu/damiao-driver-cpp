#include <iostream>

#include "damiao_driver/damiao_driver.hpp"

int main()
{
  try
  {
    dm::CanBus bus("can0");
    dm::DmMotor motor(bus, 0x01); // motor_id supports 0x01 to 0x5FF

    // Enable motor
    auto fb = motor.enable();
    std::cout << "Enabled motor " << fb.motor_id << "\n";

    // Send MIT mode command: hold position at 0 with moderate stiffness
    fb = motor.sendMit(0.0f, 0.0f, 50.0f, 1.0f, 0.0f);
    std::cout << "Position: " << fb.position << " rad, "
              << "Velocity: " << fb.velocity << " rad/s, "
              << "Torque: " << fb.torque << " Nm\n";
    std::cout << "T_MOS: " << static_cast<int>(fb.t_mos) << " C, "
              << "T_Rotor: " << static_cast<int>(fb.t_rotor) << " C\n";

    // Disable motor
    motor.disable();
    std::cout << "Motor disabled\n";
  }
  catch (const std::exception &e)
  {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
