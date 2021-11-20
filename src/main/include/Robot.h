// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/AnalogInput.h>
#include <frc/Spark.h>
#include "SFDrive.h"
#include <frc/Solenoid.h>
#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <adi/ADIS16448_IMU.h>

class Robot : public frc::TimedRobot {
 public:
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

  static const int leftLeadDeviceID = 12; // 12
  static const int leftFollowDeviceID = 13;
  static const int rightLeadDeviceID = 15; // 15
  static const int rightFollowDeviceID = 14;

  double left_y = 0.0;
  double right_x = 0.0;

  int maxPSI;
  float PSI;
  float var_input;
  bool reached_max_pressure = false;
  bool pressed_button_pressure = false;

  frc::AnalogInput * analog_input = new frc::AnalogInput(1);

  frc::Spark *compressor;

  rev::CANSparkMax* m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  rev::CANEncoder m_leftEncoder = m_leftLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
  rev::CANEncoder m_rightEncoder = m_rightLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);

  frc::Joystick* m_stick = new frc::Joystick{0};

  SFDrive* m_robotDrive = new SFDrive(m_leftLeadMotor, m_rightLeadMotor);

  frc::Solenoid valve{0};

private:
  frc::SendableChooser<std::string> m_autoChooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  frc::ADIS16448_IMU m_imu{};
  frc::SendableChooser<std::string> m_yawChooser;
  const std::string kYawDefault = "Z-Axis";
  const std::string kYawXAxis = "X-Axis";
  const std::string kYawYAxis = "Y-Axis";
  std::string m_yawSelected;
  bool m_runCal = false;
  bool m_configCal = false;
  bool m_reset = false;
  bool m_setYawAxis = false;
  frc::ADIS16448_IMU::IMUAxis m_yawActiveAxis = frc::ADIS16448_IMU::IMUAxis::kZ;

};