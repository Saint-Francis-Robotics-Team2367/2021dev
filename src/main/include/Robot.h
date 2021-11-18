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
#include <frc/DriverStation.h>

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

  void checkMotorIDs();

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

void Robot::checkMotorIDs(){
  int count = 0;
  const int maxNumIDs = 16;
  std::string workingMotorIDString = "Working Motor IDs: ";
  std::list<int> motorList;
  std::list<int>::iterator currentID;

    for (int i = 0; i < maxNumIDs; i++) {
      motorList.push_back(i);
      std::cout << "TestInit: Motor ID: " << i << std::endl;      
    } 

    for (currentID = motorList.begin(); currentID != motorList.end(); currentID++) {
      std::cout << "TestPeriodic: Testing Motor ID: " << *currentID << std::endl;
  
      rev::CANSparkMax* motor = new rev::CANSparkMax(*currentID, rev::CANSparkMax::MotorType::kBrushless);

      motor->Set(0.5);
    
      // for some reason GetFault() is needed for GetLastError() to catch the error - need to investigate
      motor->GetFault(rev::CANSparkMax::FaultID::kMotorFault);
  
      if ((motor->GetLastError() == rev::CANError::kHALError)){
        std::cout << "Deleting motor with motor ID of " << *currentID << std::endl; 
        currentID = motorList.erase(currentID);
        currentID--;
        
      } else {
        std::cout << "Working motor ID " << *currentID << " is kept in list" << std::endl;
        motor->Set(0);
      }

      delete motor;
    
    }
    std::cout << "Done iterating through list" << std::endl;
    
    std::cout << "Working motor ID ";

    for (auto &j : motorList) {
    std::cout << j << " ";
    workingMotorIDString.append(std::to_string(j) + " ");
    }
    std::cout << std::endl;
    std::cout << workingMotorIDString << std::endl;
    frc::DriverStation::ReportError(workingMotorIDString);
    std::cout << std::endl;
    
    std::cout << "Done printing working motorID list" << std::endl;
    std::cout << "Deleting list..." << std::endl;
    motorList.~list<int>();

    exit(0);
  }