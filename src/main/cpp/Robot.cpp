// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

void Robot::PIDTesting() {
  //frc::SmartDashboard::PutNumber("delta", delta);
  double currentLeftLead = m_leftLeadMotor->GetOutputCurrent();
  double currentRightLead = m_rightLeadMotor->GetOutputCurrent();
  frc::SmartDashboard::PutNumber("Total Current", currentLeftLead+currentRightLead);

  //Making it so you can manually set m_p and positionTotal: m_p is essential with PID, change by an order of magnitude to start run
  double m_P = frc::SmartDashboard::GetNumber("Pd", 0.30);
  //bool isNegative;
  m_leftLeadMotor->GetPIDController().SetP(m_P);
  m_rightLeadMotor->GetPIDController().SetP(m_P);
  frc::SmartDashboard::PutNumber("Pd", m_P);

  double m_D = frc::SmartDashboard::GetNumber("D Value", 0.0);
  //bool isNegative;
  m_leftLeadMotor->GetPIDController().SetD(m_D);
  m_rightLeadMotor->GetPIDController().SetD(m_D);
  frc::SmartDashboard::PutNumber("D Value", m_D);

  double m_I = frc::SmartDashboard::GetNumber("I Value", 0.0);
  //bool isNegative;
  m_leftLeadMotor->GetPIDController().SetI(m_I);
  m_rightLeadMotor->GetPIDController().SetI(m_I);
  frc::SmartDashboard::PutNumber("I Value", m_I);

 double I_Zone = frc::SmartDashboard::GetNumber("I_Zone", 0.0);
  //bool isNegative;
  m_leftLeadMotor->GetPIDController().SetIZone(I_Zone);
  m_rightLeadMotor->GetPIDController().SetIZone(I_Zone);
  frc::SmartDashboard::PutNumber("I_Zone", I_Zone);

  m_leftLeadMotor->GetPIDController().SetIZone(I_Zone);

  double waitTime = frc::SmartDashboard::GetNumber("waitTime", 4);
  frc::SmartDashboard::PutNumber("waitTime", waitTime);
  //frc::SmartDashboard::PutNumber("waitTime", waitTime);
  // positionTotal = frc::SmartDashboard::GetNumber("positionTotal", 6);
  // frc::SmartDashboard::PutNumber("positionTotal", positionTotal);
  // positionTotal = -6;
 
  double currTime = frc::Timer::GetFPGATimestamp();
  frc::SmartDashboard::PutNumber("currTime", currTime);
  frc::SmartDashboard::PutNumber("Setpoint", delta);
  if(currTime > prevTime + waitTime) {
      m_leftLeadMotor->GetPIDController().SetReference(delta, rev::ControlType::kPosition);
      m_rightLeadMotor->GetPIDController().SetReference(delta, rev::ControlType::kPosition);
      delta = delta * -1.0;
 
      frc::SmartDashboard::PutNumber("Right Encoder", m_rightEncoder.GetPosition());
      frc::SmartDashboard::PutNumber("Left Encoder", m_leftEncoder.GetPosition());
      //frc::SmartDashboard::PutNumber("Motor Current", m_leftLeadMotor->GetOutputCurrent());

      prevTime = frc::Timer::GetFPGATimestamp();
      frc::SmartDashboard::PutNumber("prevTime", prevTime);
  }
  //Testing for single graph
  // frc::SmartDashboard::PutNumber("Single Graph", m_leftEncoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Single Graph", delta);
  // frc::SmartDashboard::PutNumber("Single Graph", currentLeftLead+currentRightLead);
}

void Robot::RobotInit()
{
int driveMotorCurrentLimit = 38;
 m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();

  // Set current limit for drive motors
  m_leftLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
  m_rightLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
  m_leftFollowMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
  m_rightLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);

  // Set drive motors to brake mode
  m_leftLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_leftFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  // Set followers and inverts for drive motors
  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false);
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

}
void Robot::RobotPeriodic()
{
  //frc::SmartDashboard::PutNumber("x", stick->GetRawAxis(4));
  //frc::SmartDashboard::PutNumber("y ", -stick->GetRawAxis(1));
  // frc::SmartDashboard::PutNumber("current velocity", currentVelocity);
  // //frc::SmartDashboard::PutNumber("current position", currentPosition);
   frc::SmartDashboard::PutNumber("Right Encoder", m_rightEncoder.GetPosition());
   frc::SmartDashboard::PutNumber("Left Encoder", m_leftEncoder.GetPosition());
  // frc::SmartDashboard::PutNumber("current velocity", currentVelocity);
  // frc::SmartDashboard::PutNumber("Conversion", Robot::convertDistanceToTicks(1));
  frc::SmartDashboard::PutNumber("current velocity", currentVelocity);
  frc::SmartDashboard::PutNumber("current position", currentPosition);
  //frc::SmartDashboard::PutNumber("current position", currentPosition);
}


void Robot::AutonomousInit()
{
  //maybe init all in the constructor
  //PID tuned values for t-shirt cannon, having init here instead of SFDrive, might change later
  //double m_P = 0.23, m_I = 0.04, m_D = 1.68, iZone = 0.04;
  double m_P = 0.05, m_I = 0.0, m_D = 0, iZone = 0;
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

  m_leftEncoder.SetPositionConversionFactor(0.168); //check if this works!
  m_rightEncoder.SetPositionConversionFactor(0.168); 
  testBool = true;
  prevTime = frc::Timer::GetFPGATimestamp();
  currentPosition = 0;
  currentVelocity = 0;
}

void Robot::AutonomousPeriodic() {
//   whichOne = frc::SmartDashboard::GetBoolean("whichOne", whichOne);
//   testBool = frc::SmartDashboard::GetBoolean("boolean", testBool);
//   double radius = 0;
//   double angle = 360;
//   double endpoint = (angle / 360.0) * (radius + centerToWheel) * (2 * 3.1415);
//   frc::SmartDashboard::PutNumber("endpoint", endpoint);
//   testBool = frc::SmartDashboard::GetBoolean("boolean", testBool);
//   //double innerChord = ((angle * (radius - centerToWheel))/360.0) * (2 * 3.1415); [don't matter, just use ratio instead]


// //never use while loops unless threading
//   if(currentPosition < endpoint){
//     timeElapsed = frc::Timer::GetFPGATimestamp() - prevTime;
//     distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
//     if (distanceToDeccelerate > endpoint - currentPosition) {
//       currentVelocity -= (maxAcc * timeElapsed);
//     }
//     else //increase velocity
//     {
//       currentVelocity += (maxAcc * timeElapsed);
//       if (currentVelocity > maxVelocity)
//       {
//         currentVelocity = maxVelocity;
//       }
//     }

//     currentPosition += currentVelocity * timeElapsed;
//     if(currentPosition > endpoint) {
//       currentPosition = endpoint;
//     }
//     //same as other
   
//     double outerSetpoint = (currentPosition * 12) / (3.1415 * 5.7); // for now this is ticks (maybe rotations / gearRatio if not then)
//     double innerSetpoint = ((radius - centerToWheel)/(radius + centerToWheel)) * outerSetpoint;
    
//     frc::SmartDashboard::PutNumber("outerSet", outerSetpoint);
//     frc::SmartDashboard::PutNumber("innerSet", innerSetpoint);
//     //rotations and keep the multiply 
//     //probably multiplying by gear ratio twice
//     //ask abt while loops!
//     if(currentPosition < endpoint){
//       m_leftLeadMotor->GetPIDController().SetReference(outerSetpoint, rev::ControlType::kPosition);
//       m_rightLeadMotor->GetPIDController().SetReference(innerSetpoint, rev::ControlType::kPosition);
//     }
//      //what goes here
//     prevTime = frc::Timer::GetFPGATimestamp();
  

  // double totalFeet = 1;
  // if(currentPosition < totalFeet){

  //   timeElapsed = frc::Timer::GetFPGATimestamp() - prevTime;
  //   distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
  //   if (distanceToDeccelerate > totalFeet - currentPosition) {
  //     currentVelocity -= (maxAcc * timeElapsed);
  //   }
  //   else //increase velocity
  //   {
  //     currentVelocity += (maxAcc * timeElapsed);
  //     if (currentVelocity > maxVelocity)
  //     {
  //       currentVelocity = maxVelocity;
  //     }
  //   }

  //   currentPosition += currentVelocity * timeElapsed;
  //   if(currentPosition > totalFeet) {
  //     currentPosition = totalFeet;
  //   }

  //   //converting currentPosition to ticks? for the motor: inches / (circum) * ticks * gearboxRatio, might look at this later
  //   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //   double setpoint = (currentPosition * 12) / (3.1415 * 5.7) * 42 * (0.168); // for now this is ticks (maybe rotations / gearRatio if not then)
  //   m_leftLeadMotor->GetPIDController().SetReference(setpoint, rev::ControlType::kPosition);
  //   m_rightLeadMotor->GetPIDController().SetReference(setpoint, rev::ControlType::kPosition);
  //   prevTime = frc::Timer::GetFPGATimestamp();
     
// }
  Robot::PIDTesting();
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

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
