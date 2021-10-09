// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include "rev/CANSparkMax.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include "SFDrive.h"


class Robot : public frc::TimedRobot {
 public:
  rev::CANSparkMax * m_leftLeadMotor = new rev::CANSparkMax(12, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax * m_rightLeadMotor = new rev::CANSparkMax(15, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax * m_leftFollowMotor = new rev::CANSparkMax(13, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax * m_rightFollowMotor = new rev::CANSparkMax(14, rev::CANSparkMax::MotorType::kBrushless);

  rev::CANEncoder m_leftEncoder = m_leftLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
  rev::CANEncoder m_rightEncoder = m_rightLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);

  frc::Joystick *stick = new frc::Joystick(0);
  SFDrive* m_robotDrive = new SFDrive(m_leftLeadMotor, m_rightLeadMotor, m_leftFollowMotor, m_rightFollowMotor);

  double joystickY = 0.0; // negate Axis 1, not Axis 4
  double joystickX = 0.0;

  double prevTime;

  double distanceToDeccelerate;
  double currentVelocity;
  const double maxVelocity = 21;
  const double maxAcc = 20;
  //setpoint (in feet for now)
  double positionTotal = 6;
  //currpos
  double currentPosition;



  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  double convertDistanceToTicks(double);

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};

//actually revolutions
double Robot::convertDistanceToTicks(double feet) {
  double inches = feet * 12;
  double diameter = 5.7;
  double ticksPerRevolution = 42;
  double wheelCircumference = M_PI*diameter;
  // return (inches/wheelCircumference) * ticksPerRevolution;
  //fix
  //gearbox ratio
  return inches*wheelCircumference/(14/50*(24/40));
}


