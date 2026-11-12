// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/Joystick.h>
#include <wpi/print.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorController.h>

using namespace ctre::phoenix6;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::signals;
using namespace ctre::phoenix6::configs;

class Robot : public frc::TimedRobot
{
  ctre::phoenix6::hardware::TalonFX m_leftLeader{1};
  ctre::phoenix6::hardware::TalonFX m_leftFollower{2};
  ctre::phoenix6::hardware::TalonFX m_rightLeader{3};
  ctre::phoenix6::hardware::TalonFX m_rightFollower{4};

  frc::DifferentialDrive m_robotDrive{
      [&](double output)
      { m_leftLeader.Set(output); },
      [&](double output)
      { m_rightLeader.Set(output); }};

  frc::Joystick m_driverJoystick{0};
  frc::XboxController m_driverController{1};

public:
  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.

  int ticks = 0;
  void TeleopPeriodic() override
  {
    ticks++;

    if (m_driverController.GetXButtonPressed()) {
      wpi::print("x is pressed\n");
    }

    // out in
    
    //
    // code here
    //
    
    // up down
    
    //
    // code here
    //
  
    // print "hello world\n" at 300 ticks
    
    //
    // code here
    //

    double rotation = m_driverJoystick.GetY();
    double forward = m_driverJoystick.GetZ();

    m_robotDrive.ArcadeDrive(forward, rotation);
  }

private:
  /**
   * Configure all four motors
   */
  void ConfigureMotors()
  {

    // Create a configuration object
    TalonFXConfiguration cfg;

    // Talon configuration options can be found in the CTRE Phoenix6
    // documentation Based on the documentation we need to set the PID values
    // for velocity control mode
    cfg.Slot0.kP = 0.1;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;

    cfg.MotorOutput.Inverted = InvertedValue::CounterClockwise_Positive;

    // TODO: Set other PID values as needed

    // Apply the configuration to all four motors
    m_leftLeader.GetConfigurator().Apply(cfg);
    m_leftFollower.GetConfigurator().Apply(cfg);
    m_rightLeader.GetConfigurator().Apply(cfg);
    m_rightFollower.GetConfigurator().Apply(cfg);

    m_leftFollower.SetControl(Follower{m_leftLeader.GetDeviceID(), false});
    m_rightFollower.SetControl(
        Follower{m_rightFollower.GetDeviceID(),
                 false}); // Is the right follower following the follower?

    // m_leftLeader.SetInverted(InvertedValue::CounterClockwise_Positive);
    // m_leftFollower.SetInverted(InvertedValue::CounterClockwise_Positive);
    // m_rightLeader.SetInverted(-InvertedValue::Clockwise_Positive);
    // m_rightFollower.SetInverted(-InvertedValue::Clockwise_Positive);

    m_leftLeader.SetNeutralMode(NeutralModeValue::Brake);
    m_leftFollower.SetNeutralMode(NeutralModeValue::Brake);
    m_rightLeader.SetNeutralMode(NeutralModeValue::Brake);
    m_rightFollower.SetNeutralMode(NeutralModeValue::Brake);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
