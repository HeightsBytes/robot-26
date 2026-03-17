// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/POVButton.h>
#include <cameraserver/CameraServer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

RobotContainer::RobotContainer() {
  ConfigureDriverButtons();
  ConfigureOperatorButtons();

  m_chooser.SetDefaultOption("None", "None");
  m_chooser.AddOption("leave", "leave");

  frc::CameraServer::StartAutomaticCapture();

  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);

  m_drive.SetDefaultCommand(DefaultDrive(
    &m_drive, [this] { return m_driverController.GetLeftX(); },
    [this] { return m_driverController.GetLeftY(); },
    [this] { return m_driverController.GetRightX(); },
    [this] { return m_driverController.GetRightTriggerAxis(); }));
}

void RobotContainer::ConfigureDriverButtons() {
  /*
  m_driverController.LeftTrigger()
    .OnTrue(m_shooter.SetShooterTargetCMD(ShooterSubsystem::State::kShooting))
    .OnFalse(m_shooter.SetShooterTargetCMD(ShooterSubsystem::State::kStopped));
  frc2::POVButton(&m_operatorController.GetHID(), 90).OnTrue(m_shooter.AddShooterPowerCMD(.2));
  frc2::POVButton(&m_operatorController.GetHID(), 270).OnTrue(m_shooter.AddShooterPowerCMD(-.2));
  frc2::POVButton(&m_operatorController.GetHID(), 0).OnTrue(m_shooter.AddShooterPowerCMD(.1));
  frc2::POVButton(&m_operatorController.GetHID(), 180).OnTrue(m_shooter.AddShooterPowerCMD(-.1));
  */

  m_driverController.Start().OnTrue(m_drive.ResetGyroCMD());
  m_driverController.A()
    .OnTrue(m_intake.SetPivotTargetCMD(IntakeSubsystem::PivotState::kUp));
  m_driverController.B()
    .OnTrue(m_intake.SetPivotTargetCMD(IntakeSubsystem::PivotState::kDown));

  m_driverController.LeftBumper()
    .OnTrue(m_intake.SetIntakeTargetCMD(IntakeSubsystem::IntakeState::kIntaking))
    .OnFalse(m_intake.SetIntakeTargetCMD(IntakeSubsystem::IntakeState::kStopped));
}
void RobotContainer::ConfigureOperatorButtons() {
  m_operatorController.RightTrigger()
  .OnTrue(m_shooter.SetShooterTargetCMD(ShooterSubsystem::State::kShooting))
  .OnFalse(m_shooter.SetShooterTargetCMD(ShooterSubsystem::State::kStopped));

  frc2::POVButton(&m_operatorController.GetHID(), 90).OnTrue(m_shooter.AddShooterPowerCMD(.2));
  frc2::POVButton(&m_operatorController.GetHID(), 270).OnTrue(m_shooter.AddShooterPowerCMD(-.2));
  frc2::POVButton(&m_operatorController.GetHID(), 0).OnTrue(m_shooter.AddShooterPowerCMD(.1));
  frc2::POVButton(&m_operatorController.GetHID(), 180).OnTrue(m_shooter.AddShooterPowerCMD(-.1));

  m_operatorController.B().OnTrue(m_shooter.TimesShooterPowerCMD(-1));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  auto selected = m_chooser.GetSelected();
    if (selected == "None") {
      return frc2::cmd::None();
   } else {
      return pathplanner::PathPlannerAuto(selected).ToPtr();
  }
}
