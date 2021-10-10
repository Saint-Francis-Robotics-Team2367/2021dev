// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>


void Robot::RobotInit() {
  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false);
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();
}
void Robot::RobotPeriodic() {
  //frc::SmartDashboard::PutNumber("x", stick->GetRawAxis(4));
  //frc::SmartDashboard::PutNumber("y ", -stick->GetRawAxis(1));
  frc::SmartDashboard::PutNumber("current velocity", currentVelocity);
  frc::SmartDashboard::PutNumber("current position", currentPosition);
  frc::SmartDashboard::PutNumber("Right Encoder", m_rightEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Left Encoder", m_leftEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("positionTotal", positionTotal);
  frc::SmartDashboard::PutNumber("current velocity", currentVelocity);
  frc::SmartDashboard::PutNumber("current position", currentPosition);
}

void Robot::AutonomousInit() {
  
  //wait so if it doesnt show me the rest of the values it gives, its probabl the PID
  //double m_P = 0.05, m_I = 0.05, m_D = 0.1, kMaxOutput = 0.25, kMinOutput = -0.25;
  //m_P starts at 0.0005, but then is changed in periodic
  double m_P = 0.0005, m_I = 0.000, m_D = 0.0, kMaxOutput = 0.25, kMinOutput = -0.25;
  frc::SmartDashboard::PutNumber("Pd", m_P);
  frc::SmartDashboard::PutNumber("Total Position", 1);
  
  //or do I set this to 0
  //Set feet here
  m_leftLeadMotor->GetPIDController().SetP(m_P);
  m_leftLeadMotor->GetPIDController().SetI(m_I);
  m_leftLeadMotor->GetPIDController().SetD(m_D);
  //m_leftLeadMotor->GetPIDController().SetOutputRange(kMinOutput, kMaxOutput);

  m_rightLeadMotor->GetPIDController().SetP(m_P);
  m_rightLeadMotor->GetPIDController().SetI(m_I);
  m_rightLeadMotor->GetPIDController().SetD(m_D);
  //m_rightLeadMotor->GetPIDController().SetOutputRange(kMinOutput, kMaxOutput);

  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
  
  // 15:1 reduction (assumptions), with a 5.7 Diameter wheel
  m_leftEncoder.SetPositionConversionFactor(14/50*(24/40));
  m_rightEncoder.SetPositionConversionFactor(14/50*(24/40));
  prevTime = frc::Timer::GetFPGATimestamp();
  currentPosition = 0;
  currentVelocity = 0;
  
  
  
}
void Robot::AutonomousPeriodic() {
  //Making it so you can manually set m_p and positionTotal: m_p is essential with PID, change by an order of magnitude to start run
  double m_P = frc::SmartDashboard::GetNumber("Pd", 0.1);
  positionTotal = frc::SmartDashboard::GetNumber("Total Positon", 6);
  m_leftLeadMotor->GetPIDController().SetP(m_P);
  m_rightLeadMotor->GetPIDController().SetP(m_P);
  
  //Does this work or is there a substantial delay in the init call
  if(currentPosition < positionTotal) {
    double timeElapsed = frc::Timer::GetFPGATimestamp() - prevTime;

  distanceToDeccelerate = (3*currentVelocity*currentVelocity)/(2*maxAcc);
  
  //If the amount of distance we have is less than distance to deccelerate, reduce velocity, by the most possible
  if(distanceToDeccelerate > positionTotal - currentPosition) {
      //I'm pretty sure once we get to the point, the robot will completely just start going backwards or whatever
      //what do I do to make it stop, or am I already handling that -> I think I am but just make sure
      currentVelocity -= (maxAcc * timeElapsed);
  } else {
    //Else increase velocity
      currentVelocity += (maxAcc * timeElapsed);
      if(currentVelocity > maxVelocity) {
        currentVelocity = maxVelocity;
      }
  //update the current position using velocity
  //Could do encoders, but manual is pretty good
  }

  //Wouldn't this just always be positive
  currentPosition += currentVelocity * timeElapsed;

    //in rotations
    m_leftLeadMotor->GetPIDController().SetReference(0, rev::ControlType::kPosition);
    m_rightLeadMotor->GetPIDController().SetReference(10 , rev::ControlType::kPosition);
      //I know that it will always go a little above the feetNeeded, Ill fix it later

  prevTime = frc::Timer::GetFPGATimestamp();
  }
  
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
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
int main() {
  return frc::StartRobot<Robot>();
}
#endif
