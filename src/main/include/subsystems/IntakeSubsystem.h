// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <rev/SparkMax.h>
#include <rev/SparkFlex.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/SparkMaxAlternateEncoder.h>
#include <units/angle.h>
#include <units/power.h>

#include "Constants.h"


class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  enum class PivotState{
    kSwitching,
    kUp,
    kDown
  };
  enum class IntakeState{
    kIntaking,
    kStopped
  };

  IntakeSubsystem();
  void Periodic() override;

  // util functions
  double GetPivotAngle() const{
    return m_pivotEncoder.GetPosition();
  }
  bool AtPivotTarget();

  void SetPivotTarget(PivotState state){
    m_pivotTarget = state;
  }
  void SetIntakeTarget(IntakeState state){
    m_intakeTarget = state;
  };

  PivotState GetPivotTarget() const {
    return m_pivotTarget;
  }
  PivotState GetPivotActual() const {
    return m_pivotActual;
  }
  IntakeState GetIntakeTarget() const {
    return m_intakeTarget;
  }

  frc2::CommandPtr SetIntakeTargetCMD(IntakeState state){
    return this->RunOnce([this, state]{SetIntakeTarget(state); });
  }
  frc2::CommandPtr SetPivotTargetCMD(PivotState state){
    return this->RunOnce([this, state]{SetPivotTarget(state); });
  }

 private:
  double StateToOutput(PivotState state) const;
  double StateToOutput(IntakeState state) const;

  void CheckState();

  std::string ToStr(PivotState state) const;
  std::string ToStr(IntakeState state) const;

  // motors
  rev::spark::SparkFlex m_intake;
  rev::spark::SparkFlex m_pivot1;
  rev::spark::SparkFlex m_pivot2;

  rev::spark::SparkMaxConfig m_pivot1Config;
  rev::spark::SparkMaxConfig m_pivot2Config;
  rev::spark::SparkFlexConfig m_intakeConfig;

  rev::spark::SparkRelativeEncoder m_pivotEncoder;
  rev::spark::SparkClosedLoopController m_pivot1Controller;
  rev::spark::SparkClosedLoopController m_pivot2Controller;
  // end motors

  PivotState m_pivotActual;
  PivotState m_pivotTarget;

  IntakeState m_intakeTarget;
};
