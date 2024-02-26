// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Don't define as TEST_SWERVE_BOT if not using the testing swerve robot
// #define TEST_SWERVE_BOT

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/util/Color8Bit.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <photon/PhotonPoseEstimator.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <functional>
#include <map>
#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 3.9_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2 * std::numbers::pi};

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration
#ifdef TEST_SWERVE_BOT
constexpr units::meter_t kTrackWidth =
    0.5588_m;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    0.5588_m;  // Distance between centers of front and back wheels on robot
#else
constexpr units::meter_t kTrackWidth =
    0.66_m;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    0.66_m;  // Distance between centers of front and back wheels on robot
#endif
// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;

// SPARK MAX CAN IDs
constexpr int kFrontRightDrivingCanId = 2;
constexpr int kRearRightDrivingCanId = 4;
constexpr int kFrontLeftDrivingCanId = 8;
constexpr int kRearLeftDrivingCanId = 6;

constexpr int kFrontRightTurningCanId = 1;
constexpr int kRearRightTurningCanId = 3;
constexpr int kFrontLeftTurningCanId = 7;
constexpr int kRearLeftTurningCanId = 5;

constexpr units::second_t kLoopTime = 0.022_s;
constexpr double kLoopsPerSecond = 1_s / kLoopTime;
}  // namespace DriveConstants

namespace ModuleConstants {
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).

#ifdef TEST_SWERVE_BOT
constexpr int kDrivingMotorPinionTeeth = 13;
#else
constexpr int kDrivingMotorPinionTeeth = 14;
#endif

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction =
    (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor =
    (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
    (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.04;
constexpr double kDrivingI = 0;
constexpr double kDrivingD = 0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 1;
constexpr double kTurningI = 0;
constexpr double kTurningD = 0;
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::CANSparkMax::IdleMode kDrivingMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;
constexpr rev::CANSparkMax::IdleMode kTurningMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 50_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

constexpr double kSnapToAngleP = 4;
constexpr double kSnapToAngleI = 0;
constexpr double kSnapToAngleD = 0;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

const std::string kDefaultAutoName = "Leave Wing";

const auto PathConfig = pathplanner::HolonomicPathFollowerConfig(
    pathplanner::PIDConstants(0.9, 0.0,
                              0.0),            // Translation PID constants
    pathplanner::PIDConstants(0.8, 0.0, 0.0),  // Rotation PID constants
    3.0_mps,                                   // Max module speed, in m/s
#ifdef TEST_SWERVE_BOT
    0.4579874_m,  // Drive base radius in meters. Distance from robot center to
#endif
#ifndef TEST_SWERVE_BOT
    0.529844_m,
#endif
    // furthest module.
    pathplanner::ReplanningConfig()  // Default path replanning config.
                                     // See the API for the options here),
);

namespace Locations {

enum class ApproxLocation { Scoring = 0, Source, Central };

constexpr frc::Pose2d ApproxScoringLocation =
    frc::Pose2d{2.90_m, 5.58_m, 0_deg};
constexpr frc::Pose2d ApproxSourceLocation = frc::Pose2d{5.38_m, 1.5_m, 0_deg};
constexpr frc::Pose2d ApproxCentralLocation = frc::Pose2d{8.3_m, 4.11_m, 0_deg};

enum class FinalLocation {
  // Shooting in the speaker from close
  Subwoofer = 0,
  // Shooting against the amp
  Amp,
  // Shooting in the speaker from afar
  Podium,
  // Closest to alliance wall
  Source1,
  // Center
  Source2,
  // Farthest from alliance wall
  Source3,
  // Farthest climb location from alliance wall
  CenterStage,
  // Climb location on the left as seen from alliance station
  // Closest to the amp
  StageLeft,
  // Climb location on the left as seen from alliance station
  // Closest to the opposing source
  StageRight,
};

const std::map<ApproxLocation, const frc::Pose2d&> OnTheFlyPoses{
    {ApproxLocation::Scoring, ApproxScoringLocation},
    {ApproxLocation::Central, ApproxCentralLocation},
    {ApproxLocation::Source, ApproxSourceLocation}};

const std::map<FinalLocation, std::string> PoseToPath{
    // Note 2 = ApproxScoringLocation
    // Wing Line 2 = ApproxSource
    {FinalLocation::Subwoofer, "Note 2 to Speaker"},
    {FinalLocation::Amp, "Note 2 to Amp"},
    {FinalLocation::Podium, "Note 2 to Podium"},
    {FinalLocation::StageLeft, "Note 2 to Stage Left"},
    {FinalLocation::Source1, "Wing Line 2 to Source 1"},
    {FinalLocation::Source2, "Wing Line 2 to Source 2"},
    {FinalLocation::Source3, "Wing Line 2 to Source 3"},
    {FinalLocation::CenterStage, "Center Field to Center Stage"},
    {FinalLocation::StageRight, "Wing Line 2 to Stage Right"}};

struct FixtureLocation {
  frc::Pose2d location;
  units::degree_t desiredRotation;
};

const std::vector<FixtureLocation> RedFixtureLocations{
    // Podium (ScoreSpeaker)
    {.location = frc::Pose2d(13.8_m, 4.12_m, frc::Rotation2d(0_deg)),
     .desiredRotation = -148_deg},
    // Amp (ScoreAmp)
    {.location = frc::Pose2d(14.72_m, 7.85_m, frc::Rotation2d(0_deg)),
     .desiredRotation = 90_deg},
    // Speaker (ScoreSubwoofer)
    {.location = frc::Pose2d(15.39_m, 5.59_m, frc::Rotation2d(0_deg)),
     .desiredRotation = 0_deg},
};
const std::vector<FixtureLocation> BlueFixtureLocations{
    // Podium (ScoreSpeaker)
    {.location = frc::Pose2d(2.73_m, 4.11_m, frc::Rotation2d(0_deg)),
     .desiredRotation = -32_deg},
    // Amp (ScoreAmp)
    {.location = frc::Pose2d(1.83_m, 7.85_m, frc::Rotation2d(0_deg)),
     .desiredRotation = 90_deg},
    // Speaker (ScoreSubwoofer)
    {.location = frc::Pose2d(1.19_m, 5.57_m, frc::Rotation2d(0_deg)),
     .desiredRotation = 175_deg},
};
}  // namespace Locations
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;
constexpr double kDriveDeadband = 0.05;
constexpr double kVibrationIntensity = 1;
}  // namespace OIConstants

constexpr uint8_t kLedAddress = 23;
constexpr units::second_t kConnectorXDelay = 0.002_s;

// Motor IDs
namespace CANSparkMaxConstants {
constexpr int kRightIntakeSpinnyBoiId = 23;
constexpr int kLeftIntakeSpinnyBoiId = 20;
constexpr int kVectorSpinnyBoiId = 22;
constexpr int kAmpLowerSpinnyBoiId = 24;
constexpr int kAmpUpperSpinnyBoiId = 21;
constexpr int kSpeakerLowerSpinnyBoiId = 25;
constexpr int kSpeakerUpperSpinnyBoiId = 19;
constexpr int kPigeonCanId = 9;

constexpr int kTicksPerMotorRotation = 42;
}  // namespace CANSparkMaxConstants

namespace IntakingConstants {
// Change these to match actual values
constexpr double kIntakeSpeed = -0.5;
constexpr double kFeedAmpSpeed = -0.5;
constexpr double kFeedSpeakerSpeed = -1;
constexpr double kFeedSubwooferSpeed = -1;

constexpr double kOutakeSpeed = 0.5;

constexpr double kSecondaryIntakeOutSpeed = 0.05;

constexpr uint8_t kUpperBeamBreakDigitalPort = 3;
constexpr uint8_t kLowerBeamBreakDigitalPort = 2;

}  // namespace IntakingConstants

namespace ScoringConstants {
constexpr double kFreeSpinCurrentThreshold = 90;
// constexpr double kMaxSpinRpm = 5676;
constexpr double kMaxSpinRpm = 6784;

// Positive = clockwise
constexpr double kVectorSpeed = 0.1;

// These need to be different
// TODO: CHANGE TO VELOCITY RATHER THAN % OUTPUT
constexpr double kAmpLowerSpeed = 0.254;  //.264
constexpr double kAmpUpperSpeed = 0.168;  //.278

// These should match
// TODO: CHANGE TO VELOCITY RATHER THAN % OUTPUT
constexpr double kSpeakerLowerSpeed = -1;
constexpr double kSpeakerUpperSpeed = kSpeakerLowerSpeed;

// These should also match
// TODO: CHANGE TO VELOCITY RATHER THAN % OUTPUT
constexpr double kSubwooferLowerSpeed = 0.75;
constexpr double kSubwooferUpperSpeed = kSubwooferLowerSpeed;

constexpr double kScoringOutakeUpperSpeed = -0.2;
constexpr double kScoringOutakeLowerSpeed = kScoringOutakeUpperSpeed;

enum class ScoreState {
  FlywheelRamp,
  Feeding,
  Shooting,
};

constexpr units::second_t kFlywheelRampDelay = 1_s;

namespace ScoringPID {
constexpr double kSpeakerUpperP = 6e-5;
constexpr double kSpeakerUpperI = 1e-6;
constexpr double kSpeakerUpperD = 0;
constexpr double kSpeakerUpperIZone = 0;
constexpr double kSpeakerUpperFF = 0.000015;
constexpr double kSpeakerUpperVelocity = -1;

constexpr double kSpeakerLowerP = 6e-5;
constexpr double kSpeakerLowerI = 1e-6;
constexpr double kSpeakerLowerD = 0;
constexpr double kSpeakerLowerIZone = 0;
constexpr double kSpeakerLowerFF = 0.000015;
constexpr double kSpeakerLowerVelocity = -1;

constexpr double kAmpUpperP = 6e-5;
constexpr double kAmpUpperI = 1e-6;
constexpr double kAmpUpperD = 0;
constexpr double kAmpUpperIZone = 0;
constexpr double kAmpUpperFF = 0.000015;
constexpr double kAmpUpperVelocity = 0.168;

constexpr double kAmpLowerP = 6e-5;
constexpr double kAmpLowerI = 1e-6;
constexpr double kAmpLowerD = 0;
constexpr double kAmpLowerIZone = 0;
constexpr double kAmpLowerFF = 0.000015;
constexpr double kAmpLowerVelocity = 0.254;

constexpr double kSubwooferUpperP = 6e-5;
constexpr double kSubwooferUpperI = 1e-6;
constexpr double kSubwooferUpperD = 0;
constexpr double kSubwooferUpperIZone = 0;
constexpr double kSubwooferUpperFF = 0.000015;
constexpr double kSubwooferUpperVelocity = 0.95;

constexpr double kSubwooferLowerP = 6e-5;
constexpr double kSubwooferLowerI = 1e-6;
constexpr double kSubwooferLowerD = 0;
constexpr double kSubwooferLowerIZone = 0;
constexpr double kSubwooferLowerFF = 0.000015;
constexpr double kSubwooferLowerVelocity = 0.95;

const std::string kSpeakerUpperName = "Speaker Upper";
const std::string kSpeakerLowerName = "Speaker Lower";
const std::string kAmpUpperName = "Amp Upper";
const std::string kAmpLowerName = "Amp Lower";
const std::string kSubwooferUpperName = "Subwoofer Upper";
const std::string kSubwooferLowerName = "Subwoofer Lower";
}  // namespace ScoringPID

namespace SpeakerPID {
constexpr double kP = 6e-5;
constexpr double kI = 1e-6;
constexpr double kD = 0;
constexpr double kIZone = 0;
constexpr double kFF = 0.000015;
}  // namespace SpeakerPID

namespace AmpUpperPID {
constexpr double kP = 6e-5;
constexpr double kI = 1e-6;
constexpr double kD = 0;
constexpr double kIZone = 0;
constexpr double kFF = 0.000015;
}  // namespace AmpUpperPID

namespace AmpLowerPID {
constexpr double kP = 6e-5;
constexpr double kI = 1e-6;
constexpr double kD = 0;
constexpr double kIZone = 0;
constexpr double kFF = 0.000015;
}  // namespace AmpLowerPID
}  // namespace ScoringConstants

namespace ArmConstants {
// Motor Constants
constexpr int kTicksPerMotorRotation = 42;
constexpr int kReverseRotationsSoftLimit = 0;
constexpr double kAntiGravityPercentage = -0.05;

// Homing Speeds
constexpr double kRotationHomingSpeed = .15;
constexpr double kExtenderHomingSpeed = .66;
constexpr double kWristHomingSpeed = .33;

// Wrist Constants
constexpr int kWristLimitSwitchPort = 0;
constexpr int kWristGearRatio = 125;
constexpr auto kWristDegreeLimit = 144;
constexpr double kWristStepSize = 4;

constexpr double kWristSetP = 0.0018386;
constexpr double kWristSetI = 0.0075;
constexpr double kWristSetD = 0.00062724;
constexpr double kWristSetIZone = 0.01;
constexpr double kWristSetFF = 0.000015;
}  // namespace ArmConstants

// TODO: CREATE ACTUAL ROBOT VALUES FOR THESE
namespace VisionConstants {
static constexpr std::string_view kFrontCamera{"PhotonVision"};
static constexpr std::string_view kRearCamera{"Photonvision2"};
static const frc::Transform3d kRobotToCam{
    frc::Translation3d{5.296_in, 0_in, 23.892_in},
    frc::Rotation3d{0_deg, -52.541_deg, 0_deg}};
static const frc::Transform3d kRobotToCam2{
    frc::Translation3d{2.651_in, 0_in, 23.252_in},
    frc::Rotation3d{0_deg, -24.85_deg, 180_deg}};
constexpr photon::PoseStrategy kPoseStrategy =
    photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR;
static const frc::AprilTagFieldLayout kTagLayout{
    frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo)};
static const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4, 4, 8};
static const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 1};
}  // namespace VisionConstants

namespace ClimbConstants {
// These are placeholder values, these all will be changed
constexpr int kClimberLeftMotorId = 10;
constexpr int kClimberRightMotorId = 11;

constexpr double kClimberSetP = 1;
constexpr double kClimberSetI = 0;
constexpr double kClimberSetD = 0;

// Maximum arm extension distance
constexpr double kMaxArmDistance = 1000;
// Arm climbing position
constexpr double kClimbExtensionPosition = 5;
// Arm retracted position
constexpr double kClimbRetractPosition = 3;
constexpr double kInPerRotation = 0.1;

constexpr double kClimbStepSize = 1;
constexpr double kClimbHomingSpeed = 1;
constexpr int kTicksPerMotorRotation = 42;

constexpr double kCLimberExtendSpeed = 1;

// Distance between left and right arm centers
constexpr units::meter_t kClimberOffsetDistance = 4_m;

}  // namespace ClimbConstants

namespace RobotConstants {
#ifdef TEST_SWERVE_BOT
const std::string kRoborioSerialNumber = "032B4B68";
#else
const std::string kRoborioSerialNumber = "0326F2F2";
#endif
}  // namespace RobotConstants

enum class RobotState {
  Manual = 0,
  ScoringSpeaker,
  ScoringAmp,
  ScoringSubwoofer,
  Loaded,
  Intaking,
  ClimbStageLeft,
  ClimbStageCenter,
  ClimbStageRight,
  SourceLeft,
  SourceCenter,
  SourceRight,
  Funni,
  AutoSequenceSpeaker,
  AutoSequenceAmp,
  AutoSequenceSubwoofer,
};

typedef std::function<RobotState()> StateGetter;
