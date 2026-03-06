// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <rev/SparkFlex.h>
#include <rev/config/SparkFlexConfig.h>
#include <units/angle.h>
#include <units/power.h>

#include "Constants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  enum class State {
    kShooting,
    kStopped,
  };

  ShooterSubsystem();

  void Periodic() override;

  void SetTarget(State state){
     m_target = state;
  };

  State GetTarget() const {
    return m_target;
  }

  frc2::CommandPtr SetShooterTargetCMD(State state){
    return this->RunOnce([this, state]{SetTarget(state); });
  }

  frc2::CommandPtr AddShooterPowerCMD(double amount){
    return this->RunOnce([this, amount]{ kPower += amount; });
  }
  frc2::CommandPtr TimesShooterPowerCMD(double amount){
    return this->RunOnce([this, amount]{ kPower *= amount; });
  }
  



 private:
  double StateToOutput(State state) const;

  void CheckState();

  std::string ToStr(State state) const;

  // motors
  rev::spark::SparkFlex m_shooter;
  rev::spark::SparkFlexConfig m_shooterConfig;
  
  // states
  State m_target;

  // unfortunate consquence of time
  double kPower;
};