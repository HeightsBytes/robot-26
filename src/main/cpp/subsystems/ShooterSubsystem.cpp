// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include "utils/Util.h"
#include <frc2/command/Command.h>

ShooterSubsystem::ShooterSubsystem() :
    m_shooter(ShooterConstants::kShooterMotorPort, rev::spark::SparkFlex::MotorType::kBrushless),
    m_target(State::kStopped) {

        m_shooterConfig
            .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kCoast)
            .Inverted(false);

        m_shooter.Configure(m_shooterConfig, rev::spark::SparkFlex::ResetMode::kResetSafeParameters, rev::spark::SparkFlex::PersistMode::kPersistParameters);
    }   

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {

    if(m_target == State::kStopped){
        m_shooter.StopMotor();
    } else if(m_target == State::kShooting){
        m_shooter.Set(.7); // fix this at grizzy
    }

}

std::string ShooterSubsystem::ToStr(State state) const{
    using enum State;
    
    switch(state){
        case kShooting:
            return "Shooting";
            break;
        case kStopped:
            return "Stopped";
            break;
    }
}