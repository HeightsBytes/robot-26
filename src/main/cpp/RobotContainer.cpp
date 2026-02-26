// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureDriverButtons();
  ConfigureOperatorButtons();

  m_drive.SetDefaultCommand(DefaultDrive(
    &m_drive, [this] { return m_driverController.GetLeftX(); },
    [this] { return m_driverController.GetLeftY(); },
    [this] { return m_driverController.GetRightX(); },
    [this] { return m_driverController.GetRightTriggerAxis(); }));
}

void RobotContainer::ConfigureDriverButtons() {}
void RobotContainer::ConfigureOperatorButtons() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
