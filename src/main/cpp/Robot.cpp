// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();

  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);

  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false);
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

  m_autoChooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_autoChooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  m_yawChooser.SetDefaultOption(kYawDefault, kYawDefault);
  m_yawChooser.AddOption(kYawXAxis, kYawXAxis);
  m_yawChooser.AddOption(kYawYAxis, kYawYAxis);
  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);
  frc::SmartDashboard::PutData("IMUYawAxis", &m_yawChooser);
  frc::SmartDashboard::PutBoolean("RunCal", false);
  frc::SmartDashboard::PutBoolean("ConfigCal", false);
  frc::SmartDashboard::PutBoolean("Reset", false);
  frc::SmartDashboard::PutBoolean("SetYawAxis", false);
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", -(m_stick->GetRawAxis(1)));
  frc::SmartDashboard::PutNumber("right x: ", m_stick->GetRawAxis(4));

  frc::SmartDashboard::PutNumber("YawAngle", m_imu.GetAngle());

  if (m_imu.GetAngle() < 360) {
      m_robotDrive->ArcadeDrive(0.16, 0.16);
  }
  frc::SmartDashboard::PutNumber("XCompAngle", m_imu.GetXComplementaryAngle());
  frc::SmartDashboard::PutNumber("YCompAngle", m_imu.GetYComplementaryAngle());
  m_runCal = frc::SmartDashboard::GetBoolean("RunCal", false);
  m_configCal = frc::SmartDashboard::GetBoolean("ConfigCal", false);
  m_reset = frc::SmartDashboard::GetBoolean("Reset", false);
  m_setYawAxis = frc::SmartDashboard::GetBoolean("SetYawAxis", false);
  m_yawSelected = m_yawChooser.GetSelected();

  // Set IMU settings
  /*if (m_configCal) {
    m_imu.ConfigCalTime(8);
    m_configCal = frc::SmartDashboard::PutBoolean("ConfigCal", false);
  }
  if (m_reset) {
    m_imu.Reset();
    m_reset = frc::SmartDashboard::PutBoolean("Reset", false);
  }
  if (m_runCal) {
    m_imu.Calibrate();
    m_runCal = frc::SmartDashboard::PutBoolean("RunCal", false);
  }*/
  
  // Read the desired yaw axis from the dashboard
  if (m_yawSelected == "X-Axis") {
    m_yawActiveAxis = frc::ADIS16448_IMU::IMUAxis::kX;
  }
  else if (m_yawSelected == "Y-Axis") {
    m_yawActiveAxis = frc::ADIS16448_IMU::IMUAxis::kY;
  }
  else {
    m_yawActiveAxis = frc::ADIS16448_IMU::IMUAxis::kZ;
  }
  // Set the desired yaw axis from the dashboard
  if (m_setYawAxis) {
    m_imu.SetYawAxis(m_yawActiveAxis);
    m_setYawAxis = frc::SmartDashboard::PutBoolean("SetYawAxis", false);
  }

  
}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
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
    valve.Set(false);
    pressed_button_pressure = true;
    reached_max_pressure = false;
    frc::SmartDashboard::PutBoolean("triggerpress", true);
    frc::SmartDashboard::PutBoolean("valve", false);
  }

  if ((m_stick->GetRawButtonPressed(2)) && (reached_max_pressure)) {
    valve.Set(true);
    frc::SmartDashboard::PutBoolean("valve", true);
  }

  if (m_stick->GetRawButtonPressed(3)) {
    valve.Set(false);
    frc::SmartDashboard::PutBoolean("valve", false);
  }

  if ((!reached_max_pressure) && (pressed_button_pressure)) {
    if (PSI < maxPSI) {
      frc::SmartDashboard::PutNumber("currPSI", PSI);
      compressor->Set(1);
    } else {
      compressor->Set(0);
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

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
