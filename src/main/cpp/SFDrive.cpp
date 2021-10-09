// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SFDrive.h"
#include <math.h>
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
