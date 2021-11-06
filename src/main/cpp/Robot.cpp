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
  for (int i; i < 16; i++) {
    motorList.push(i);
    std::cout << "TestInit: Motor ID: " << i << " at address " << std::endl;      
  } 

}
void Robot::TestPeriodic() {
   
  if (testedMotors == false) {
    for (int i; i < 16; i++) {
      std::cout << "TestPeriodic: Testing Motor ID: " << motorList.front() << std::endl;

      rev::CANSparkMax* motor = new rev::CANSparkMax(motorList.front(), rev::CANSparkMax::MotorType::kBrushless);

      motor->Set(0.5);
      // motorList[i]->Set(0.5);
      if (motor->GetLastError() == rev::CANError::kCantFindFirmware){
        std::cout << "Deleting motor with motor ID of " << motorList.front() << std::endl;
        delete motor;
        motorList.pop();
      } else {
        workingMotorList.push_back(motorList.front());
        delete motor;
        motorList.pop();
      }

      if (motorList.empty()) {
        std::cout << "Done iterating through list" << std::endl;

        for (auto i : workingMotorList)
          std::cout << "Working motor ID " << i << std::endl;
        
        std::cout << "Done printing working motorID list" << std::endl;

        exit(-1);
      }
      //   motorList[i]= 0; 
        
      //   std::cout << "Removed Motor ID " << i << "at address " << &motorList[i] << std::endl;
      // }
        
    }
  } 
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
