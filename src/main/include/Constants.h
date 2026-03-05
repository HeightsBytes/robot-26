#pragma once

#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <pathplanner/lib/path/PathConstraints.h>
//#include <pathplanner/lib/config/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

namespace DriveConstants {
    namespace CanIds{
        inline constexpr int kFrontLeftDriveMotorPort = 7; 
        inline constexpr int kRearLeftDriveMotorPort = 2;
        inline constexpr int kFrontRightDriveMotorPort = 8;
        inline constexpr int kRearRightDriveMotorPort = 4 ;

        inline constexpr int kFrontLeftTurningMotorPort = 6;
        inline constexpr int kRearLeftTurningMotorPort = 3;
        inline constexpr int kFrontRightTurningMotorPort = 10;
        inline constexpr int kRearRightTurningMotorPort = 5;

        inline constexpr int kFrontLeftTurningEncoderPorts = 1;
        inline constexpr int kRearLeftTurningEncoderPorts = 2;
        inline constexpr int kFrontRightTurningEncoderPorts = 3;
        inline constexpr int kRearRightTurningEncoderPorts = 4;

        inline constexpr int kPidgeonID = 20;
    } // namespace CanIds
  inline constexpr double kFrontRightOffset = 107.31 - 180;
  inline constexpr double kFrontLeftOffset = 228.25;
  inline constexpr double kRearRightOffset = 193.45 - 180;
  inline constexpr double kRearLeftOffset = 128.76; // - 180 if inverted

  inline constexpr bool kFrontRightInverted = true;
  inline constexpr bool kFrontLeftInverted = true;
  inline constexpr bool kRearRightInverted = true;
  inline constexpr bool kRearLeftInverted = true;

  inline constexpr auto kMaxChassisSpeed = 4.25_mps;
  inline constexpr auto kMaxAngularSpeed =
    units::radians_per_second_t(1 * std::numbers::pi);
  inline constexpr auto kMaxAngularAcceleration =
    units::radians_per_second_squared_t(2 * std::numbers::pi);

  inline constexpr auto kTrackWidth = 0.31369_m;
  inline constexpr auto kTrackLength = 0.31369_m;

  inline frc::SwerveDriveKinematics<4> kDriveKinematics{ // ++, +-, -+, --
    frc::Translation2d(kTrackLength, kTrackWidth),
    frc::Translation2d(kTrackLength, -kTrackWidth),
    frc::Translation2d(-kTrackLength, kTrackWidth),
    frc::Translation2d(-kTrackLength, -kTrackWidth)};
} // namespace DriveConstants

namespace ModuleConstants{
  inline constexpr double kGearRatio = 1 / 6.75;
  inline constexpr double kWheelDiameterMeters = 0.05092958;
  inline constexpr double kDriveEncoderDistancePerPulse =
    kGearRatio * 2 * std::numbers::pi * kWheelDiameterMeters;

  inline constexpr double kTurnRatio = 7.0 / 150.0;
  inline constexpr double kTurnEncoderRatio = kTurnRatio * 2.0 * std::numbers::pi;

  inline constexpr double kPDrive = 0.175;
  inline constexpr double kIDrive = 0;
  inline constexpr double kDDrive = 0.02;
  inline constexpr double kFFDrive = 2.67;

  inline constexpr double kPTurn = 1.25;
  inline constexpr double kITurn = 0;
  inline constexpr double kDTurn = 0;
  inline constexpr double kFFTurn = 0;

  inline constexpr auto kMaxModuleSpeed = 4.5_mps;
} // namespace ModuleConstants

namespace VisionConstants {
inline const frc::Transform3d RightTransform{
    frc::Translation3d(-15_in, -7_in, 24_in),
    frc::Rotation3d{0_deg, 0_deg, -150_deg}};
inline const frc::Transform3d LeftTransform{
    frc::Translation3d(-15_in, 7_in, 24_in),
    frc::Rotation3d{0_deg, 0_deg, 150_deg}};
}  // namespace VisionConstants

namespace AutoConstants {
  inline constexpr auto kMaxSpeed = 3_mps;
  inline constexpr auto kMaxAcceleration = 3_mps_sq;
  inline constexpr auto kMaxAngularSpeed =
      units::radians_per_second_t(std::numbers::pi);
  inline constexpr auto kMaxAngularAcceleration =
      units::radians_per_second_squared_t(std::numbers::pi);

  inline constexpr pathplanner::PIDConstants kPIDTranslation{1.25, 0, 0.07};
  inline constexpr pathplanner::PIDConstants kPIDRotation{1, 0, 0.1};
  inline static pathplanner::RobotConfig kConfig = pathplanner::RobotConfig::fromGUISettings();

/*
  inline constexpr pathplanner::HolonomicPathFollowerConfig kConfig{
      kPIDTranslation, kPIDRotation, kMaxSpeed,
      0.53881_m, /**std::sqrt(2 * 15in ^ 2)**//*
      pathplanner::ReplanningConfig()};
*/

  inline constexpr pathplanner::PathConstraints kConstraints{
      kMaxSpeed, kMaxAcceleration, kMaxAngularSpeed, kMaxAngularAcceleration};
} // namespace AutoConstants

namespace OIConstants {
  inline constexpr int kDriverControllerPort = 0;
  inline constexpr int kOperatorControllerPort = 1;
}  // namespace OIConstants

namespace ShooterConstants{
  inline constexpr int kShooterMotorPort = 1000;
}

namespace IntakeConstants{
  inline constexpr int kIntakeMotorPort = 1000;
  inline constexpr int kPivot1MotorPort = 1000;
  inline constexpr int kPivot2MotorPort = 1000;

  inline constexpr double kPivotEncoderRatio = 1;

  inline constexpr int kP = 0;
  inline constexpr int kI = 0;
  inline constexpr int kD = 0;
  inline constexpr double kMinOutput = -0.1;
  inline constexpr double kMaxOutput = 0.1;

  namespace Speeds{
    inline constexpr double kStopped = 0;
    inline constexpr double kIntaking = 0.5;
  }
  namespace PivotPositions{
    inline constexpr double kUp = 0;
    inline constexpr double kDown = 0;
    inline constexpr double kTolerance = 0;
  }
}