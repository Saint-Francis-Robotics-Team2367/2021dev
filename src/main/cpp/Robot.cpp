// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

// roboRIO-TEAM-frc.local

void Robot::RobotInit() {
  // m_leftLeadMotor->RestoreFactoryDefaults();
  // m_rightLeadMotor->RestoreFactoryDefaults();
  // m_leftFollowMotor->RestoreFactoryDefaults();
  // m_rightFollowMotor->RestoreFactoryDefaults();

  // m_leftEncoder.SetPosition(0);
  // m_rightEncoder.SetPosition(0);

  // m_leftLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_rightLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_leftFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_rightFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  // m_leftLeadMotor->SetInverted(true);
  // m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  // m_rightLeadMotor->SetInverted(false); 
  // m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

  std::cout << "Robot::RobotInit filename: " << filename << std::endl;
  motorData.open(filename);
  
  if (motorData) {
    motorData << "count,left_input_speed,right_input_speed,speed_left_lead,speed_left_follow,speed_right_lead,speed_right_follow,left_motor_equal,right_motor_equal" << std::endl;
    std::cout << "Robot::RobotInit: wrote headers in file" << std::endl;
    
    motorData.close();
  } else {
    std::cout << "Robot::RobotInit: unable to open file " << filename << std::endl;
  }
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", -(m_stick->GetRawAxis(1)));
  frc::SmartDashboard::PutNumber("right x: ", m_stick->GetRawAxis(4));
}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

/* class motorIDCheck {
  public:
    int motorID;
  
  rev::CANSparkMax* m_Motor = new rev::CANSparkMax(motorID, rev::CANSparkMax::MotorType::kBrushless);

  if (error(Set(0.5))) {
    return 0
  } else {
    return 1
  }
} */

void Robot::TeleopInit() {
  frc::Solenoid valve{0};
  // m_leftEncoder.SetPosition(0);
  // m_rightEncoder.SetPosition(0);
  compressor = new frc::Spark(1);
  pressed_button_pressure = true;
  valve.Set(false);
}
void Robot::TeleopPeriodic() {
  left_y = m_stick->GetRawAxis(1);
  right_x = m_stick->GetRawAxis(4);

  // m_robotDrive->ArcadeDrive(-left_y, right_x);

  analog_input->GetVoltage();
  frc::SmartDashboard::PutNumber("analogInput", analog_input->GetVoltage());

  var_input = frc::SmartDashboard::GetNumber("varInput", 1);
  frc::SmartDashboard::PutNumber("varInput", var_input);

  maxPSI = frc::SmartDashboard::GetNumber("maxPSI", 82);
  frc::SmartDashboard::PutNumber("maxPSI", maxPSI);

  PSI = (analog_input->GetVoltage()) * 100 + 10; // transfer function
  if (m_stick->GetRawButtonPressed(1)) {
    // valve.Set(false);
    pressed_button_pressure = true;
    reached_max_pressure = false;
    frc::SmartDashboard::PutBoolean("triggerpress", true);
    frc::SmartDashboard::PutBoolean("valve", false);
  }

  if ((m_stick->GetRawButtonPressed(2)) && (reached_max_pressure)) {
    // valve.Set(true);
    frc::SmartDashboard::PutBoolean("valve", true);
  }

  if (m_stick->GetRawButtonPressed(3)) {
    // valve.Set(false);
    frc::SmartDashboard::PutBoolean("valve", false);
  }

  if ((!reached_max_pressure) && (pressed_button_pressure)) {
    if (PSI < maxPSI) {
      frc::SmartDashboard::PutNumber("currPSI", PSI);
      // compressor->Set(1);
    } else {
      // compressor->Set(0);
      reached_max_pressure = true;
      pressed_button_pressure = false; 
      frc::SmartDashboard::PutBoolean("triggerpress", false);
    }
  }
  
  // if button pressed,
  // include if statement
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  testedMotors = false;


  for (int i = 0; i < 16; i++) {
    motorList.push_back(i);
    std::cout << "TestInit: Motor ID: " << i << std::endl;      
  } 
  
}
void Robot::TestPeriodic() {
   
  if (testedMotors == false) {
    for (currentID = motorList.begin(); currentID != motorList.end(); currentID++) {
      std::cout << "TestPeriodic: Testing Motor ID: " << *currentID << std::endl;
  
      rev::CANSparkMax* motor = new rev::CANSparkMax(*currentID, rev::CANSparkMax::MotorType::kBrushless);

      motor->Set(0.5);
      
      if (count % 30) {
        std::cout << "Waiting in the count..." << std::endl;
      }
      // for some reason GetFault() is needed for GetLastError() to catch the error - need to investigate
      motor->GetFault(rev::CANSparkMax::FaultID::kMotorFault);
  
      if ((motor->GetLastError() == rev::CANError::kHALError)){
        std::cout << "Deleting motor with motor ID of " << *currentID << std::endl; 
        currentID = motorList.erase(currentID);
        
      } else {
        std::cout << "Working motor ID " << *currentID << " is kept in list" << std::endl;
        
      }

      delete motor;

      count++;
    }
    std::cout << "Done iterating through queue" << std::endl;
    
    testedMotors = true;    

    } else {
      std::cout << "Working motor ID ";

      for (auto &j : motorList) {
      std::cout << j << " ";
      }

      std::cout << std::endl;
      
    std::cout << "Done printing working motorID list" << std::endl;
    exit(0);
    }
      //   motorList[i]= 0; 
        
      //   std::cout << "Removed Motor ID " << i << "at address " << &motorList[i] << std::endl;
      // }
    
    // testedMotors = true;
  // } else {
  //   std::cout << "Done testing motor IDs" << std::endl;
  //   std::cout << "Printing list..." << std::endl;
  //   for (int i; i < 16; i++) {
  //     std::cout << motorList[i] << " ";   
  //   }
  //   exit(-1);
  // }
  
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
