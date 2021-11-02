// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

void Robot::RobotInit()
{
  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false);
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();
}
void Robot::RobotPeriodic()
{
  //frc::SmartDashboard::PutNumber("x", stick->GetRawAxis(4));
  //frc::SmartDashboard::PutNumber("y ", -stick->GetRawAxis(1));
  frc::SmartDashboard::PutNumber("current velocity", currentVelocity);
  //frc::SmartDashboard::PutNumber("current position", currentPosition);
  frc::SmartDashboard::PutNumber("Right Encoder", m_rightEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Left Encoder", m_leftEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("current velocity", currentVelocity);
  frc::SmartDashboard::PutNumber("Conversion", Robot::convertDistanceToTicks(1));
  //frc::SmartDashboard::PutNumber("current position", currentPosition);
}


void Robot::AutonomousInit()
{
  //maybe init all in the constructor
  //PID tuned values for t-shirt cannon, having init here instead of SFDrive, might change later
  double m_P = 0.23, m_I = 0.04, m_D = 1.68, iZone = 0.04;

  m_leftLeadMotor->GetPIDController().SetP(m_P);
  m_leftLeadMotor->GetPIDController().SetI(m_I);
  m_leftLeadMotor->GetPIDController().SetD(m_D);
  m_leftLeadMotor->GetPIDController().SetIZone(iZone);

  m_rightLeadMotor->GetPIDController().SetP(m_P);
  m_rightLeadMotor->GetPIDController().SetI(m_I);
  m_rightLeadMotor->GetPIDController().SetD(m_D);
  m_rightLeadMotor->GetPIDController().SetIZone(iZone);

  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);

  m_leftEncoder.SetPositionConversionFactor(14 / 50 * (24 / 40)); //check if this works!
  m_rightEncoder.SetPositionConversionFactor(14 / 50 * (24 / 40)); 

}

void Robot::AutonomousPeriodic() {
  if(testBool) {
    //m_robotDrive->PIDTuning(1);
    m_robotDrive->PIDDrive(3, 7, 21);
    testBool = false;
  }
  
}

void Robot::TeleopInit() {

}
void Robot::TeleopPeriodic()
{
  // suggest putting this code into one single method in a new file b/c it's very messy for TeleopPeriodic

  joystickY = -stick->GetRawAxis(1); // negate Axis 1, not Axis 4
  joystickX = stick->GetRawAxis(4);
  m_robotDrive->ArcadeDrive(joystickX, joystickY);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
