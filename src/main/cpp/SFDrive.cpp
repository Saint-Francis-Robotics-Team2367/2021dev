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