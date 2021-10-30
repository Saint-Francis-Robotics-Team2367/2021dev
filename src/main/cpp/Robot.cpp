// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

// roboRIO-TEAM-frc.local

void Robot::RobotInit() {
  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();

  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);

  m_leftLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_leftFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false); 
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

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
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
  compressor = new frc::Spark(1);
  pressed_button_pressure = true;
  valve.Set(false);
}
void Robot::TeleopPeriodic() {
  left_y = m_stick->GetRawAxis(1);
  right_x = m_stick->GetRawAxis(4);

  m_robotDrive->ArcadeDrive(-left_y, right_x);

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
 
  int motorList[16];

  testedMotors = false;

}
void Robot::TestPeriodic() {
  
  if (testedMotors == false) {
    for (int i; i < 16; i++) {
      rev::CANSparkMax* motorList[i] = new rev::CANSparkMax(i, rev::CANSparkMax::MotorType::kBrushless);
      std::cout << "Motor ID: " << i << "at address " << &motorList[i] << std::endl;
      motorList[i]->Set(0.5);
      std::cout << "GetLastError message" << motorList[i]->GetLastError() << std::endl;;
/*       if (motorList[i]->GetLastError() != ""){
        delete motorList[i]; 
        
        std::cout << "Removed Motor ID " << i << "at address " << &motorList[i] << std::endl;
      }
        
      } */
    }
    testedMotors = true;
    std::cout << "Done testing motor IDs" << std::endl;
    std::cout << "Printing list..." << std::endl;
    for (int item : motorList) {
      std::cout << item << " ";
    }
  }
  
  
  left_inputSpeed = frc::SmartDashboard::GetNumber("setLeft", 0);
  right_inputSpeed = frc::SmartDashboard::GetNumber("setRight", 0);

  frc::SmartDashboard::PutNumber("setLeft", left_inputSpeed);
  frc::SmartDashboard::PutNumber("setRight", right_inputSpeed);

  m_leftLeadMotor->Set(left_inputSpeed);
  m_rightLeadMotor->Set(right_inputSpeed);

   // sleep(10);

  speed_leftLead = m_leftLeadMotor->Get();
  frc::SmartDashboard::PutNumber("speed_leftLead", speed_leftLead);

  speed_leftFollow = m_leftFollowMotor->Get();
  frc::SmartDashboard::PutNumber("speed_leftFollow", speed_leftFollow);

  speed_rightLead = m_rightLeadMotor->Get();
  frc::SmartDashboard::PutNumber("speed_rightLead", speed_rightLead);

  speed_rightFollow = m_rightFollowMotor->Get();
  frc::SmartDashboard::PutNumber("speed_rightFollow", speed_rightFollow);

  if (speed_leftLead == speed_leftFollow) {
    leftMotor_equal = true;
  }

  if (speed_rightLead == speed_rightFollow) {
    rightMotor_equal = true;
  }

  frc::SmartDashboard::PutBoolean("leftMotor_equal", leftMotor_equal);
  frc::SmartDashboard::PutBoolean("rightMotor_equal", rightMotor_equal);

  count++;
  
  /* if (count % 50 == 0) {
    motorData.open(filename, std::ios::app); // open with append
  
    if (motorData) {
      std::cout << "Robot::TestPeriodic [" << count << "] logging motor values to file" << std::endl;
    
      motorData << count << ",";
      motorData << left_inputSpeed << ",";
      motorData << right_inputSpeed << ",";
      motorData << speed_leftLead << ",";
      motorData << speed_leftFollow << ",";
      motorData << speed_rightLead << ",";
      motorData << speed_rightFollow << ",";
      motorData << leftMotor_equal << ",";
      motorData << rightMotor_equal << std::endl;
    
      motorData.close();

    } else {
      std::cout << "Robot::TestPeriodic file not found" << std::endl;
      exit(-1);
    }
  } */
  
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
