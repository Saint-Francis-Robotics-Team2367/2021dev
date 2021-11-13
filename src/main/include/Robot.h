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
#include <frc/commands/WaitCommand.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <ctime>
#include <list>
#include <Error.h>
#include <queue>

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

  // static const int leftLeadDeviceID = 12; // 15 for 2367 // 3 for 8109
  // static const int leftFollowDeviceID = 13; // 14
  // static const int rightLeadDeviceID = 13; // 12 // 12 for 8109
  // static const int rightFollowDeviceID = 14; // 13

  double left_y = 0.0;
  double right_x = 0.0;

  int maxPSI;
  float PSI;
  float var_input;
  bool reached_max_pressure = false;
  bool pressed_button_pressure = true;

  float speed_leftLead;
  float speed_leftFollow;
  float speed_rightLead;
  float speed_rightFollow;

  float left_inputSpeed;
  float right_inputSpeed;

  bool leftMotor_equal;
  bool rightMotor_equal;

  int count;

  bool testedMotors;

  const int maxNumIDs = 17;

  std::list<int> motorList;
  std::list<int>::iterator currentID;


  std::string filename = "/home/lvuser/logdata.csv";

  frc::AnalogInput * analog_input = new frc::AnalogInput(1);

  frc::Spark *compressor;

  // rev::CANSparkMax* m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  // rev::CANSparkMax* m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  // rev::CANSparkMax* m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  // rev::CANSparkMax* m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  // rev::CANEncoder m_leftEncoder = m_leftLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
  // rev::CANEncoder m_rightEncoder = m_rightLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);

  frc::Joystick* m_stick = new frc::Joystick{0};

  // SFDrive* m_robotDrive = new SFDrive(m_leftLeadMotor, m_rightLeadMotor);

  // frc::Solenoid valve{0};

  // std::ofstream motorData;
};