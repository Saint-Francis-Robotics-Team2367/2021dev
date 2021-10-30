// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SFDrive.h"
#include <math.h>
#include <frc/Timer.h>
//Maybe smart dashboard if I want, could gobal variable, h file thing too if I want (figure out)


//Review class construction*
SFDrive::SFDrive(rev::CANSparkMax* leftLeadMotor, rev::CANSparkMax* rightLeadMotor, 
    rev::CANSparkMax* leftFollowMotor, rev::CANSparkMax* rightFollowMotor) : leftLeadMotor{leftLeadMotor}, rightLeadMotor{rightLeadMotor},
    leftFollowMotor{leftFollowMotor}, rightFollowMotor{rightFollowMotor} {}


void SFDrive::ArcadeDrive(double joystickX, double joystickY) {
  double afterLeftDeadband;
  double afterRightDeadband;
  double leftMotorOutput;
  double rightMotorOutput;


  if (fabs(joystickX) <= deadband)
    joystickX = 0;
  if (fabs(joystickY) <= deadband)
    joystickY = 0;
  
  double leftAbs = std::fabs(joystickX);
  double rightAbs = std::fabs(joystickY);
  
  // scaling

  //reason for the if statements is that if leftAbs is 0 when its under deadband, the input becomes negative, so it fixes it
  if (leftAbs != 0)
    afterLeftDeadband = (1/(1-deadband)) * leftAbs - (deadband/(1/deadband));
  else
    afterLeftDeadband = 0;

  if (rightAbs != 0)
    afterRightDeadband = (1/(1-deadband)) * rightAbs - (deadband/(1/deadband));
  else
    afterRightDeadband = 0;
  
  joystickX = std::copysign(pow(afterLeftDeadband, 2), joystickX);
  joystickY = std::copysign(pow(afterRightDeadband, 2), joystickY);

  //To fix turning backwards
  if (joystickY >= 0.0) {
    leftMotorOutput = joystickY + joystickX;
    rightMotorOutput = joystickY - joystickX;
  }
  else {
    leftMotorOutput = joystickY - joystickX;
    rightMotorOutput = joystickY + joystickX;
  }


  leftLeadMotor->Set(-leftMotorOutput);
  //negate here
  rightLeadMotor->Set(rightMotorOutput);
}

void SFDrive::PIDDrive(float totalFeet, float maxAcc, float maxVelocity) {
  //forward movement only *implement backwards movement with if statement if necessary
  float currentPosition, currentVelocity, timeElapsed, distanceToDeccelerate, setpoint = 0; //currentPosition is the set point
  float prevTime = frc::Timer::GetFPGATimestamp();
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
  while(currentPosition < totalFeet){
    timeElapsed = frc::Timer::GetFPGATimestamp() - prevTime;
    distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
    if (distanceToDeccelerate > totalFeet - currentPosition) {
      currentVelocity -= (maxAcc * timeElapsed);
    }
    else //increase velocity
    {
      currentVelocity += (maxAcc * timeElapsed);
      if (currentVelocity > maxVelocity)
      {
        currentVelocity = maxVelocity;
      }
    }

    currentPosition += currentVelocity * timeElapsed;
    if(currentPosition > totalFeet) {
      currentPosition = totalFeet;
    }

    //converting currentPosition to ticks? for the motor: inches / (circum) * ticks * gearboxRatio, might look at this later
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    setpoint = (currentPosition * 12) / (PI * 5.7) * 42 * (0.168); // for now this is ticks (maybe rotations / gearRatio if not then)
    leftLeadMotor->GetPIDController().SetReference(setpoint, rev::ControlType::kPosition);
    rightLeadMotor->GetPIDController().SetReference(-setpoint, rev::ControlType::kPosition);
    prevTime = frc::Timer::GetFPGATimestamp();
  }
}

void SFDrive::PIDTurn(float angle, float radius, float maxAcc, float maxVelocity) {
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
  float currentPosition, currentVelocity, endpoint, setpoint, timeElapsed, distanceToDeccelerate = 0; //currentPosition is the set point
  float prevTime = frc::Timer::GetFPGATimestamp();
  endpoint = ((angle/360.0) * (2 * PI * radius));



  while(currentPosition < endpoint){
    timeElapsed = frc::Timer::GetFPGATimestamp() - prevTime;
    distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
    if (distanceToDeccelerate > endpoint - currentPosition) {
      currentVelocity -= (maxAcc * timeElapsed);
    }
    else //increase velocity
    {
      currentVelocity += (maxAcc * timeElapsed);
      if (currentVelocity > maxVelocity)
      {
        currentVelocity = maxVelocity;
      }
    }

    currentPosition += currentVelocity * timeElapsed;
    if(currentPosition > endpoint) {
      currentPosition = endpoint;
    }

    //converting currentPosition to ticks? for the motor: inches / (circum) * ticks * gearboxRatio, might look at this later
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    setpoint = (currentPosition * 12) / (PI * 5.7) * 42 * (0.168); // for now this is ticks (maybe rotations / gearRatio if not then)
    leftLeadMotor->GetPIDController().SetReference(setpoint, rev::ControlType::kPosition);
    rightLeadMotor->GetPIDController().SetReference(0, rev::ControlType::kPosition); //what goes here
    prevTime = frc::Timer::GetFPGATimestamp();

    //Overall questions -> What goes in setReference second, is this right, but the robot is standing still right, 
    //so during path planning we call PID turning to move a certain angle, but it won't turn in place?, how do we want it to turn
    //because we have different radius's so the angle turns a different distance each time along the circle, is that what we're trying
    //not just in place
    //ask on saturday ig
  }
  //we should have some inner set point though right?
}



void SFDrive::PIDTuning(float delta) {
  double prevTime = frc::Timer::GetFPGATimestamp();
  double currentLeftLead = leftLeadMotor->GetOutputCurrent();
  double currentRightLead = rightLeadMotor->GetOutputCurrent();
  frc::SmartDashboard::PutNumber("Total Current", currentLeftLead+currentRightLead);

  //Making it so you can manually set m_p and positionTotal: m_p is essential with PID, change by an order of magnitude to start run
  double m_P = frc::SmartDashboard::GetNumber("Pd", 0.23);
  //bool isNegative;
  leftLeadMotor->GetPIDController().SetP(m_P);
  rightLeadMotor->GetPIDController().SetP(m_P);
  frc::SmartDashboard::PutNumber("Pd", m_P);

  double m_D = frc::SmartDashboard::GetNumber("D Value", 1.68);
  //bool isNegative;
  leftLeadMotor->GetPIDController().SetD(m_D);
  rightLeadMotor->GetPIDController().SetD(m_D);
  frc::SmartDashboard::PutNumber("D Value", m_D);

  double m_I = frc::SmartDashboard::GetNumber("I Value", 0.04);
  //bool isNegative;
  leftLeadMotor->GetPIDController().SetI(m_I);
  rightLeadMotor->GetPIDController().SetI(m_I);
  frc::SmartDashboard::PutNumber("I Value", m_I);

 double I_Zone = frc::SmartDashboard::GetNumber("I_Zone", 0.04);
  //bool isNegative;
  leftLeadMotor->GetPIDController().SetIZone(I_Zone);
  rightLeadMotor->GetPIDController().SetIZone(I_Zone);
  frc::SmartDashboard::PutNumber("I_Zone", I_Zone);

  leftLeadMotor->GetPIDController().SetIZone(I_Zone);

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
      leftLeadMotor->GetPIDController().SetReference(delta, rev::ControlType::kPosition);
      delta = delta * -1.0;
      rightLeadMotor->GetPIDController().SetReference(delta, rev::ControlType::kPosition);
 
      frc::SmartDashboard::PutNumber("Right Encoder", m_rightEncoder.GetPosition());
      frc::SmartDashboard::PutNumber("Left Encoder", m_leftEncoder.GetPosition());

      prevTime = frc::Timer::GetFPGATimestamp();
      frc::SmartDashboard::PutNumber("prevTime", prevTime);
  }
}


void SFDrive::setP(double value)
{
    leftLeadMotor->GetPIDController().SetP(value);
    rightLeadMotor->GetPIDController().SetP(value);
}

void SFDrive::setI(double value)
{
    leftLeadMotor->GetPIDController().SetI(value);
    rightLeadMotor->GetPIDController().SetI(value);
}

void SFDrive::setD(double value)
{
    leftLeadMotor->GetPIDController().SetD(value);
    rightLeadMotor->GetPIDController().SetD(value);
}