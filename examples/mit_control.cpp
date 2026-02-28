#include <iostream>

#include "damiao_driver/damiao_driver.hpp"

int main()
{
  try
  {
    // Create a CAN bus instance for "can0" (singleton per interface)
    auto bus = dm::CanBus::create("can0");

    // Create a DM-J4310 motor with ID 0x01
    dm::DmMotor motor_j4310(bus, 0x01, dm::MotorType::DM_J4310);

    // Create a DM-J4340 motor on the same bus with ID 0x02
    dm::DmMotor motor_j4340(bus, 0x02, dm::MotorType::DM_J4340);

    // Requesting the same interface again returns the same bus instance
    auto bus_same = dm::CanBus::create("can0");
    // bus_same == bus (same shared_ptr target)

    // Enable motor
    auto fb = motor_j4310.enable();
    std::cout << "Enabled motor " << fb.motor_id << "\n";

    // Send MIT mode command: hold position at 0 with moderate stiffness
    fb = motor_j4310.sendMit(0.0f, 0.0f, 50.0f, 1.0f, 0.0f);
    std::cout << "Position: " << fb.position << " rad, "
              << "Velocity: " << fb.velocity << " rad/s, "
              << "Torque: " << fb.torque << " Nm\n";
    std::cout << "T_MOS: " << static_cast<int>(fb.t_mos) << " C, "
              << "T_Rotor: " << static_cast<int>(fb.t_rotor) << " C\n";

    // Disable motor
    motor_j4310.disable();
    std::cout << "Motor disabled\n";
  }
  catch (const std::exception &e)
  {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
