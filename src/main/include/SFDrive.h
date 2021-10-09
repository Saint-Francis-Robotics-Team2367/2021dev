// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "rev/CANSparkMax.h"

class SFDrive {
 public:
  // member variables
  const double deadband = 0.08;
  rev::CANSparkMax* leftLeadMotor = nullptr;
  rev::CANSparkMax* rightLeadMotor = nullptr;
  rev::CANSparkMax* leftFollowMotor = nullptr;
  rev::CANSparkMax* rightFollowMotor = nullptr;

  // constructor
  SFDrive(rev::CANSparkMax* leftLeadMotor, rev::CANSparkMax* rightLeadMotor, rev::CANSparkMax* leftFollowMotor, rev::CANSparkMax* rightFollowMotor);
  
 public:
  // methods
  void ArcadeDrive(double joystickX, double joystickY);
};
