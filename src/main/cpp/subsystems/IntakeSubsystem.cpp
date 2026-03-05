// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() :
    m_intake(IntakeConstants::kIntakeMotorPort, rev::spark::SparkFlex::MotorType::kBrushless),
    m_pivot1(IntakeConstants::kPivot1MotorPort, rev::spark::SparkFlex::MotorType::kBrushless),
    m_pivot2(IntakeConstants::kPivot2MotorPort, rev::spark::SparkFlex::MotorType::kBrushless),

    m_pivotEncoder(m_pivot1.GetEncoder()),
    m_pivot1Controller(m_pivot1.GetClosedLoopController()),
    m_pivot2Controller(m_pivot2.GetClosedLoopController()),

    m_pivotActual(PivotState::kSwitching),
    m_pivotTarget(PivotState::kUp),
    m_intakeTarget(IntakeState::kStopped) {

        m_pivot1Config
            .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kBrake)
            .Inverted(false)
            .SmartCurrentLimit(30);
        m_pivot2Config
            .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kBrake)
            .Inverted(false)
            .SmartCurrentLimit(30);
        m_intakeConfig
            .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kCoast)
            .Inverted(false);
        m_pivot1Config.encoder
            .PositionConversionFactor(IntakeConstants::kPivotEncoderRatio);
        m_pivot1Config.closedLoop
            .Pid(
                IntakeConstants::kP,
                IntakeConstants::kI,
                IntakeConstants::kD
            )
            .PositionWrappingEnabled(false)
            .OutputRange(IntakeConstants::kMinOutput, IntakeConstants::kMaxOutput);
        m_pivot2Config.closedLoop
            .Pid(
                IntakeConstants::kP,
                IntakeConstants::kI,
                IntakeConstants::kD
            )
            .PositionWrappingEnabled(false)
            .OutputRange(IntakeConstants::kMinOutput, IntakeConstants::kMaxOutput);

        m_intake.Configure(m_intakeConfig, rev::spark::SparkFlex::ResetMode::kResetSafeParameters, rev::spark::SparkFlex::PersistMode::kPersistParameters);
        m_pivot1.Configure(m_pivot1Config, rev::spark::SparkFlex::ResetMode::kResetSafeParameters, rev::spark::SparkFlex::PersistMode::kPersistParameters);
        m_pivot2.Configure(m_pivot2Config, rev::spark::SparkFlex::ResetMode::kResetSafeParameters, rev::spark::SparkFlex::PersistMode::kPersistParameters);
    }


// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
    CheckState();

    m_pivot1Controller.SetSetpoint(StateToOutput(m_pivotTarget), rev::spark::SparkMax::ControlType::kPosition);
    m_pivot2Controller.SetSetpoint(StateToOutput(m_pivotTarget), rev::spark::SparkMax::ControlType::kPosition);

    if(StateToOutput(m_intakeTarget) == IntakeConstants::Speeds::kStopped){
        m_intake.StopMotor();
    } else {
        m_intake.Set(StateToOutput(m_intakeTarget));
    }
}

double IntakeSubsystem::StateToOutput(IntakeState state) const {
    using enum IntakeState;
    namespace IS = IntakeConstants::Speeds;
    
    switch(state){
        case kStopped:
            return IS::kStopped;
            break;
        case kIntaking:
            return IS::kIntaking;
            break;
        default:
            return 0;
            break;
    }
}

double IntakeSubsystem::StateToOutput(PivotState state) const {
    using enum PivotState;
    namespace PP = IntakeConstants::PivotPositions;

    switch(state){
        case kUp:
            return PP::kUp;
            break;
        case kDown:
            return PP::kDown;
            break;
        default:
            return 0;
            break;
    }
}

void IntakeSubsystem::CheckState(){
    using enum PivotState;
    namespace PP = IntakeConstants::PivotPositions;

    double angle = GetPivotAngle();

    if(frc::IsNear(PP::kUp, angle, PP::kTolerance)){
        m_pivotActual = kUp;
        return;
    }
    if(frc::IsNear(PP::kDown, angle, PP::kTolerance)){
        m_pivotActual = kDown;
        return;
    }

    m_pivotActual = kSwitching;
}