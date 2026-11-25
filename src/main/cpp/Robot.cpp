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
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_elevator{5};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_intake{6};

  frc::DifferentialDrive m_robotDrive{
      [&](double output)
      { m_leftLeader.Set(output); },
      [&](double output)
      { m_rightLeader.Set(output); }};

  frc::Joystick m_driverJoystick{0};
  frc::XboxController m_driverController{1};
  int motorTicks = 0;
  bool motorUp = false;
  bool motorDown = false;

public:
  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.

  int ticks = 0;
  void TeleopPeriodic() override
  {
    ticks++;

    if (m_driverController.GetXButtonPressed())
    {
      wpi::print("x is pressed\n");
    }

    // out in

    //
    // code here
    //

    // up down
    if (m_driverController.GetYButtonPressed())
    {
      if (motorUp == false && motorDown == false)
      {
        wpi::print("motor spin up\n");
        motorUp = true;
      }
    }
    if (motorUp == true)
    {
      motorTicks++;
      // add code to make the motor actually go up
      m_elevator.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.25);
      if (motorTicks == 250)
      {
        wpi::print("motor stop\n");
        motorUp = false;
        motorTicks = 0;
        // add code to make motor stop
        m_elevator.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      }
    }
    if (m_driverController.GetAButton())
    {
      if (motorUp == false && motorDown == false)
      {
        wpi::print("motor spin down\n");
        motorDown = true;
        m_elevator.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.25);
      }
    }
    if (motorDown == true)
    {
      motorTicks++;
      if (motorTicks == 250)
      {
        wpi::print("motor stop\n");
        motorTicks = 0;
        motorDown = false;
        m_elevator.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      }
    }

    //left trigger stuff
    
    if (m_driverController.GetLeftTriggerAxis() <= 0.4999 && m_driverController.GetLeftTriggerAxis() >= 0.0001){
      wpi::print("left trigger less than half pressed\n");
      m_intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    }
    if (m_driverController.GetLeftTriggerAxis() >= 0.5 && m_driverController.GetLeftTriggerAxis() <= 0.999)
    {
      wpi::print("left trigger over half pressed\n");
      m_intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.25);
    }
    if (m_driverController.GetLeftTriggerAxis() == 1)
    {
      wpi::print("left trigger fully pressed\n");
      m_intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.25);
    }

    //right trigger stuff
    
    if (m_driverController.GetRightTriggerAxis() <= 0.4999 && m_driverController.GetRightTriggerAxis() >= 0.0001){
      wpi::print("right trigger less than half pressed\n");
      m_intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    }
    if (m_driverController.GetRightTriggerAxis() >= 0.5 && m_driverController.GetRightTriggerAxis() <= 0.999)
    {
      wpi::print("right trigger over half pressed\n");
      m_intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.25);
    }
    if (m_driverController.GetRightTriggerAxis() == 1)
    {
      wpi::print("right trigger fully pressed\n");
      m_intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.25);
    }

    //
    // code here
    //

    // print "hello world\n" at 300 ticks

    //
    // code here
    //

    double rotation = m_driverJoystick.GetY();
    double forward = m_driverJoystick.GetZ();

     if (m_driverJoystick.GetTrigger() == true)
    {
      forward /= 2;
      rotation /= 2;
    }

    m_robotDrive.ArcadeDrive(forward/3, rotation/3);
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
