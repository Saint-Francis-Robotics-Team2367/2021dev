// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

// roboRIO-TEAM-frc.local

void Robot::RobotInit() {
  // Restore factory defaults on drive motors
  // m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  // m_leftFollowMotor->RestoreFactoryDefaults();
  // m_rightFollowMotor->RestoreFactoryDefaults();

  // Set current limit for drive motors
  // m_leftLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
  m_rightLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
  // m_leftFollowMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
  // m_rightLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);

  // Set drive motors to brake mode
  // m_leftLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_leftFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_rightFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  // Set followers and inverts for drive motors
  // m_leftLeadMotor->SetInverted(true);
  // m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false);
  // m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", -(m_stick->GetRawAxis(1)));
  frc::SmartDashboard::PutNumber("right x: ", m_stick->GetRawAxis(4));
  // frc::SmartDashboard::PutNumber("current", m_leftLeadMotor->GetOutputCurrent());
}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  frc::Solenoid valve{0};
  // m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
  compressor = new frc::Spark(1);
  pressed_button_pressure = false;
  valve.Set(false);
  
  upFlag, downFlag, maxFlag, buttonReleased = false;
}

void Robot::TeleopPeriodic() {
  left_y = m_stick->GetRawAxis(1);
  right_x = m_stick->GetRawAxis(4);

  // m_robotDrive->ArcadeDrive(-left_y, right_x);

  // analog_input->GetVoltage();
  // frc::SmartDashboard::PutNumber("analogInput", analog_input->GetVoltage());

  // var_input = frc::SmartDashboard::GetNumber("varInput", 1);
  // frc::SmartDashboard::PutNumber("varInput", var_input);

  // maxPSI = frc::SmartDashboard::GetNumber("maxPSI", 82);
  // frc::SmartDashboard::PutNumber("maxPSI", maxPSI);

  // PSI = (analog_input->GetVoltage()) * 100 + 10; // transfer function
  // if (m_stick->GetRawButtonPressed(1)) {
  //   // valve.Set(false);
  //   pressed_button_pressure = true;
  //   reached_max_pressure = false;
  //   frc::SmartDashboard::PutBoolean("triggerpress", true);
  //   frc::SmartDashboard::PutBoolean("valve", false);
  // }

  // if ((m_stick->GetRawButtonPressed(2)) && (reached_max_pressure)) {
  //   // valve.Set(true);
  //   frc::SmartDashboard::PutBoolean("valve", true);
  // }

  // if (m_stick->GetRawButtonPressed(3)) {
  //   // valve.Set(false);
  //   frc::SmartDashboard::PutBoolean("valve", false);
  // }

  // if ((!reached_max_pressure) && (pressed_button_pressure)) {
  //   if (PSI < maxPSI) {
  //     frc::SmartDashboard::PutNumber("currPSI", PSI);
  //     // compressor->Set(1);
  //   } else {
  //     // compressor->Set(0);
  //     reached_max_pressure = true;
  //     pressed_button_pressure = false; 
  //     frc::SmartDashboard::PutBoolean("triggerpress", false);
  //   }
  // }
  
  // button A -  manual up
  
  // testing one limit switch
  if (bottomLimitSwitch.Get()) {
    std::cout << "limit switch is on!" << std::endl;
    frc::SmartDashboard::PutBoolean("limitSwitch", true);
  } else {
    std::cout << "limit switch is off!" << std::endl;
    frc::SmartDashboard::PutBoolean("limitSwtich", false);
  }

  /*
  if (m_stick->GetRawButton(0)) {
    upFlag = true;
    downFlag = false;
    maxFlag = false;

    buttonReleased = false;
  }
  
  // check if manual up button is released
  if (m_stick->GetRawButtonReleased(0)) {
    buttonReleased = true;
  }

  // button B - full up
  if (m_stick->GetRawButtonPressed(1)) {
    upFlag = true;
    downFlag = false;
    maxFlag = true;
  }

  // button Y - down all
  if (m_stick->GetRawButtonPressed(3)) {
    upFlag = false;
    downFlag = true;
    maxFlag = true;
  }

  // button X - manual down
  if (m_stick->GetRawButton(2)) {
    upFlag = false;
    downFlag = true;
    maxFlag = false;

    buttonReleased = false;
  }

  // check if manual down button is released
  if (m_stick->GetRawButtonReleased(2)) {
    buttonReleased = true;
  }

  if (upFlag) {
    m_elevator->Set(1);
    if (maxFlag && topLimitSwitch.Get()) {
      m_elevator->Set(0);
    } else if ((maxFlag == false) && buttonReleased && (movingLimitSwitch.Get() || topLimitSwitch.Get())) {
      m_elevator->Set(0);
    }
  }

  if (downFlag) {
    m_elevator->Set(-1);
    if (maxFlag && bottomLimitSwitch.Get()) {
      m_elevator->Set(0);
    } else if ((maxFlag == false) && buttonReleased && (movingLimitSwitch.Get() || bottomLimitSwitch.Get())) {
      m_elevator->Set(0);
    }
  }

*/
  // if (upFlag && maxFlag && (topLimitSwitch.Get() == false)) {
  //   m_elevator->Set(1.0);
  // } else if (upFlag) {
  //   m_elevator->Set(1.0);
  // }
  // if (upFlag && m_stick->GetRawButtonReleased(0) && (movingLimitSwitch.Get() || topLimitSwitch.Get())) {
  //   m_elevator->Set(0.0);
  // }

  // if (downFlag && maxFlag && (bottomLimitSwitch.Get() == false)) {
  //   m_elevator->Set(-1.0);
  // } else if (downFlag) {
  //   m_elevator->Set(-1.0);
  // }

  // if button pressed,
  // include if statement
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  tested_motors = false;
}

void Robot::TestPeriodic() {
  if (tested_motors == false) {
    TestFunctions->checkMotorIDs();
    tested_motors = true;
  } else {
    exit(0);
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif