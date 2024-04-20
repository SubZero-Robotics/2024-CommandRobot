// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// Don't define as TEST_SWERVE_BOT if not using the testing swerve robot
// #define TEST_SWERVE_BOT

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/util/Color.h>
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
#include <string>
#include <vector>

#include "ColorConstants.h"
#include "subsystems/singleaxis/ISingleAxisSubsystem.h"

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
constexpr auto kMaxAcceleration = 1.5_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

enum class AutoType {
  EmptyAuto = 0,
  FourNoteAuto,
  PlaceAndLeave,
  ThreeNoteAuto,
  TwoNoteAuto,
  LeaveWing,
  TwoNoteCenter,
  TwoNoteSource,
  ThreeNoteCenter,
};

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

extern const frc::TrapezoidProfile<units::degree>::Constraints
    kRotationalAxisConstraints;

extern const frc::TrapezoidProfile<units::meter>::Constraints
    kLinearAxisConstraints;

const std::string kDefaultAutoName = "Leave Wing";

const std::string kScoreSubwooferName = "Score Subwoofer";
const std::string kShootSubwooferName = "Shoot Subwoofer";
const std::string kIntakeName = "Run Intake";
const std::string kLedFunniName = "LedFunni";

const auto PathConfig = pathplanner::HolonomicPathFollowerConfig(
    pathplanner::PIDConstants(3.14, 0.0,
                              0.0),             // Translation PID constants
    pathplanner::PIDConstants(3.14, 0.0, 0.0),  // Rotation PID constants
    3.0_mps,                                    // Max module speed, in m/s
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

enum class ApproxLocation {
  Scoring = 0,
  Source,
  Central,
  AllianceWing,
  EnemyWing,
};

constexpr frc::Pose2d ApproxScoringLocation =
    frc::Pose2d{2.90_m, 5.58_m, 0_deg};
constexpr frc::Pose2d ApproxSourceLocation = frc::Pose2d{5.38_m, 1.5_m, 0_deg};
constexpr frc::Pose2d ApproxCentralLocation = frc::Pose2d{8.3_m, 4.11_m, 0_deg};
// TODO
constexpr frc::Pose2d ApproxAllianceWingLocation =
    frc::Pose2d{2.82_m, 6.07_m, 0_deg};
// TODO
constexpr frc::Pose2d ApproxEnemyWingLocation =
    frc::Pose2d{13.7_m, 1.65_m, 0_deg};

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
  // Close to alliance scoring locations
  Scoring,
  // Near our source
  EnemyWing,
};

const std::map<ApproxLocation, const frc::Pose2d&> OnTheFlyPoses{
    {ApproxLocation::Scoring, ApproxScoringLocation},
    {ApproxLocation::Central, ApproxCentralLocation},
    {ApproxLocation::Source, ApproxSourceLocation},
    {ApproxLocation::AllianceWing, ApproxAllianceWingLocation},
    {ApproxLocation::EnemyWing, ApproxEnemyWingLocation}};

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
  // The location of the point to begin tracking
  frc::Pose2d fixtureLocation;
  // The distance around the fixtureLocation where tracking should be active
  units::inch_t locationRadius;
  // What the robot should "look" at
  frc::Pose2d trackedPose;
};

const std::vector<FixtureLocation> RedFixtureLocations{
    // Podium (ScoreSpeaker)
    {.fixtureLocation = frc::Pose2d(13.8_m, 4.12_m, frc::Rotation2d(0_deg)),
     .locationRadius = 4_ft,
     .trackedPose = frc::Pose2d(16.54_m, 5.6_m, frc::Rotation2d(0_deg))},
    // Amp (ScoreAmp)
    {.fixtureLocation = frc::Pose2d(14.64_m, 8.5_m, frc::Rotation2d(0_deg)),
     .locationRadius = 6_ft,
     .trackedPose = frc::Pose2d(14.75_m, 20_m, frc::Rotation2d(180_deg))},
    // Speaker (ScoreSubwoofer)
    {.fixtureLocation = frc::Pose2d(15.34_m, 5.6_m, frc::Rotation2d(0_deg)),
     .locationRadius = 6_ft,
     .trackedPose = frc::Pose2d(16.54_m, 5.6_m, frc::Rotation2d(180_deg))},
    // Feeding (Feed)
    {.fixtureLocation = frc::Pose2d(6_m, 1_m, frc::Rotation2d(0_deg)),
     // TODO: bigger radius and motor velocity changes based on distance
     .locationRadius = 2_ft,
     .trackedPose = frc::Pose2d(15_m, 7.5_m, frc::Rotation2d(0_deg))}};
const std::vector<FixtureLocation> BlueFixtureLocations{
    // Podium (ScoreSpeaker)
    // TODO
    {.fixtureLocation = frc::Pose2d(2.9_m, 4.15_m, frc::Rotation2d(0_deg)),
     .locationRadius = 4_ft,
     .trackedPose = frc::Pose2d(0_m, 5.6_m, frc::Rotation2d(0_deg))},
    // Amp (ScoreAmp)
    {.fixtureLocation = frc::Pose2d(1.82_m, 8.5_m, frc::Rotation2d(0_deg)),
     .locationRadius = 6_ft,
     .trackedPose = frc::Pose2d(1.82_m, 20_m, frc::Rotation2d(180_deg))},
    // Speaker (ScoreSubwoofer)
    {.fixtureLocation = frc::Pose2d(1.35_m, 5.6_m, frc::Rotation2d(0_deg)),
     .locationRadius = 6_ft,
     .trackedPose = frc::Pose2d(0_m, 5.6_m, frc::Rotation2d(180_deg))},
    // Feeding (Feed)
    {.fixtureLocation = frc::Pose2d(10_m, 1_m, frc::Rotation2d(0_deg)),
     // TODO: bigger radius and motor velocity changes based on distance
     .locationRadius = 2_ft,
     .trackedPose = frc::Pose2d(1_m, 7.5_m, frc::Rotation2d(0_deg))}};
}  // namespace Locations
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;
constexpr double kDriveDeadband = 0.05;
constexpr double kVibrationIntensity = 1;
}  // namespace OIConstants

// LED Constants
constexpr uint8_t kLedAddress = 23;
constexpr units::second_t kConnectorXDelay = 0.002_s;
constexpr double kAccelThreshold = 7;

// BaseSingleAxisSubsystem Constants
namespace BaseSingleAxisSubsystemConstants {
constexpr double kMotorDeadSpeedRange = 0.05;
constexpr double kMaxRotPosition = 350;
}  // namespace BaseSingleAxisSubsystemConstants

// Motor IDs
namespace CANConstants {
constexpr int kArmSpinnyBoiId = 62;
constexpr int kLeftIntakeSpinnyBoiId = 20;
constexpr int kVectorSpinnyBoiId = 22;
constexpr int kAmpLowerSpinnyBoiId = 24;
constexpr int kAmpUpperSpinnyBoiId = 21;
constexpr int kSpeakerLowerSpinnyBoiId = 25;
constexpr int kSpeakerUpperSpinnyBoiId = 19;
constexpr int kPigeonCanId1 = 9;
constexpr int kPigeonCanId2 = 13;

constexpr int kTicksPerMotorRotation = 42;
}  // namespace CANConstants

namespace IntakingConstants {
// Change these to match actual values
constexpr double kIntakeSpeed = 0.8;
// Make er' hover!
constexpr double kIntakeAutoSpeed = 0.58;
constexpr double kFeedAmpSpeed = 0.5;
constexpr double kFeedSpeakerSpeed = 1;
constexpr double kFeedSubwooferSpeed = 1;

constexpr double kOutakeSpeed = -0.5;

constexpr double kSecondaryIntakeOutSpeed = -0.05;

constexpr uint8_t kCenterLowerBeamBreakDigitalPort = 3;
constexpr uint8_t kCenterUpperBeamBreakDigitalPort = 1;
constexpr uint8_t kLowerPodiumBeamBreakDigitalPort = 2;
constexpr uint8_t kLowerampBeamBreakDigitalPort = 4;
constexpr uint8_t kUpperPodiumBeamBreakDigitalPort = 6;
constexpr uint8_t kUpperAmpBeamBreakDigitalPort = 5;

constexpr units::revolutions_per_minute_t kMaxRpm = 5676_rpm;

constexpr double kDowntakeSpeed = 0.4;

namespace IntakingPID {
constexpr double kIntakingP = 6e-5;
constexpr double kIntakingI = 1e-6;
constexpr double kIntakingD = 0;
constexpr double kIntakingIZone = 0;
constexpr double kIntakingFF = 0.000015;
}  // namespace IntakingPID

}  // namespace IntakingConstants

namespace ScoringConstants {
constexpr double kFreeSpinCurrentThreshold = 90;
// constexpr double kMaxSpinRpm = 5676;
constexpr units::revolutions_per_minute_t kMaxSpinRpm = 6784_rpm;

constexpr double kShuffleSpeed = 0.05;

// Positive = clockwise
constexpr double kVectorSpeed = -0.4;

// These need to be different
constexpr double kAmpLowerSpeed = -0.254 * 1.9;  // .264
constexpr double kAmpUpperSpeed = -0.168 * 1.4;  // .278

// These should match
constexpr double kSpeakerLowerSpeed = 1;
constexpr double kSpeakerUpperSpeed = kSpeakerLowerSpeed;

// These should also match
constexpr double kSubwooferLowerSpeed = -0.95;
constexpr double kSubwooferUpperSpeed = kSubwooferLowerSpeed;

constexpr double kFeedLowerSpeed = 0.9;
constexpr double kFeedUpperSpeed = 0.9;

constexpr double kScoringOutakeUpperSpeed = 0.2;
constexpr double kScoringOutakeLowerSpeed = kScoringOutakeUpperSpeed;

enum class ScoreState {
  FlywheelRamp,
  Feeding,
  Shooting,
};

constexpr units::second_t kFlywheelRampDelay = 0.5_s;

namespace ScoringPID {
constexpr double kSpeakerP = 6e-5;
constexpr double kSpeakerI = 1e-6;
constexpr double kSpeakerD = 0;
constexpr double kSpeakerIZone = 0;
constexpr double kSpeakerFF = 0.000015;

// constexpr double kSpeakerLowerP = 6e-5;
// constexpr double kSpeakerLowerI = 1e-6;
// constexpr double kSpeakerLowerD = 0;
// constexpr double kSpeakerLowerIZone = 0;
// constexpr double kSpeakerLowerFF = 0.000015;
// constexpr double kSpeakerLowerVelocity = -1;

constexpr double kAmpP = 6e-5;
constexpr double kAmpI = 1e-6;
constexpr double kAmpD = 0.000000;
constexpr double kAmpIZone = 0;
constexpr double kAmpFF = 0.000015;

// constexpr double kAmpLowerP = 6e-5;
// constexpr double kAmpLowerI = 1e-6;
// constexpr double kAmpLowerD = 0;
// constexpr double kAmpLowerIZone = 0;
// constexpr double kAmpLowerFF = 0.000015;
// constexpr double kAmpLowerVelocity = 0.254;

// constexpr double kSubwooferUpperP = 6e-5;
// constexpr double kSubwooferUpperI = 1e-6;
// constexpr double kSubwooferUpperD = 0;
// constexpr double kSubwooferUpperIZone = 0;
// constexpr double kSubwooferUpperFF = 0.000015;
// constexpr double kSubwooferUpperVelocity = 0.75;

// constexpr double kSubwooferLowerP = 6e-5;
// constexpr double kSubwooferLowerI = 1e-6;
// constexpr double kSubwooferLowerD = 0;
// constexpr double kSubwooferLowerIZone = 0;
// constexpr double kSubwooferLowerFF = 0.000015;
// constexpr double kSubwooferLowerVelocity = 0.75;

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

// Homing Speeds
constexpr double kRotationHomingSpeed = .15;

// Arm Constants
constexpr int kArmGearRatio = 125;
constexpr units::degree_t kArmRelativeDistancePerRev = 360_deg * (1 / 8.75);
constexpr units::degree_t kArmAbsoluteDistancePerRev = 360_deg;
constexpr units::degree_t kAmpRotation = 142_deg;
constexpr units::degree_t kMaxRotation = 190_deg;
constexpr units::degree_t kHomeRotation = 10_deg;
constexpr units::degrees_per_second_t kDefaultVelocity = 10_deg_per_s;
constexpr double kVelocityScalar = 1.0;
constexpr units::degree_t kTolerance = 2_deg;
constexpr units::meter_t kArmLength = 0.2_m;

constexpr double kArmP = 0.075;
constexpr double kArmI = 0;
constexpr double kArmD = 0;
constexpr double kArmIZone = 0;
constexpr double kArmFF = 0;

static const SingleAxisMechanism kArmMechanism = {
    // length
    0.2_m,
    // min angle
    110_deg,
    // line width
    6,
    // color
    ColorConstants::kBlue};
}  // namespace ArmConstants

namespace MechanismConstants {
constexpr double kClimberLeftX = 0.5;
constexpr double kClimberLeftY = 0;
constexpr double kClimberRightX = 0.5;
constexpr double kClimberRightY = 1;
constexpr double kArmRootX = 0.5;
constexpr double kArmRootY = 0.5;
constexpr double kArmPostX = 0.4;
constexpr units::degree_t kArmPostAngle = 90_deg;
}  // namespace MechanismConstants

namespace VisionConstants {
static constexpr std::string_view kFrontCamera{"PhotonVision"};
static constexpr std::string_view kRearCamera{"Photonvision2"};
static const frc::Transform3d kRobotToCam2{
    frc::Translation3d{2.147_in, 0_in, 23.369_in},
    frc::Rotation3d{0_deg, -23.461_deg, 180_deg}};
static const frc::Transform3d kRobotToCam{
    frc::Translation3d{5.714_in, 0_in, 23.533_in},
    frc::Rotation3d{0_deg, -23.461_deg, 0_deg}};
constexpr photon::PoseStrategy kPoseStrategy =
    photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR;
static const frc::AprilTagFieldLayout kTagLayout{
    frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo)};
static const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4, 4, 8};
static const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 1};

const std::string kLimelightName = "limelight";
constexpr double kKnownPixelWidth = 58;
constexpr units::inch_t kNoteWidth = 14_in;
constexpr units::inch_t kKnownCalibrationDistance = 60_in;
constexpr units::inch_t kCalibrationDistanceAreaPercentage =
    kKnownCalibrationDistance * kKnownPixelWidth;
constexpr auto focalLength = kCalibrationDistanceAreaPercentage / kNoteWidth;

constexpr units::degree_t kCameraAngle = -20_deg;
constexpr units::inch_t kCameraLensHeight = 15_in;
constexpr double kConfidenceThreshold = 0.3;
constexpr double kTrigDistancePercentage = 0.5;

constexpr units::degree_t kGamepieceRotation = 180_deg;
constexpr frc::Pose2d kSimGamepiecePose =
    frc::Pose2d(7_m, 4_m, frc::Rotation2d(kGamepieceRotation));
}  // namespace VisionConstants

namespace TurnToPoseConstants {
    constexpr units::degrees_per_second_t kProfileVelocity = 960_deg_per_s;
    constexpr units::degrees_per_second_squared_t kProfileAcceleration = 1200_deg_per_s_sq;
    constexpr double kTurnP = 5;
    constexpr double kTurnI = 0;
    constexpr double kTurnD = 0;
    constexpr frc::Pose2d kPoseTolerance = frc::Pose2d(0.2_m, 0.2_m, frc::Rotation2d(0.5_deg));
}

namespace ClimbConstants {
constexpr int kClimberLeftMotorId = 10;
constexpr int kClimberRightMotorId = 11;

constexpr double kClimberSetP = 40.0;
constexpr double kClimberSetI = 0.0;
constexpr double kClimberSetD = 0.0;
constexpr double kClimberSetIZone = 0.0;
constexpr double kClimberSetFF = 0.0;

// Minimum climber extension distance
constexpr units::inch_t kMinClimberDistance = 0_in;
// Maximum climber extension distance
constexpr units::inch_t kMaxClimberDistance = 36_in;
// Extended position
constexpr units::inch_t kClimbExtensionPosition = 24_in;
// Retracted position
constexpr units::inch_t kClimbRetractPosition = 1_in;
constexpr units::inch_t kInPerRotation = 1_in / 23.1;
constexpr units::inch_t kClimberTolerance = 0.5_in;

constexpr double kClimbStepSize = 1.0;
constexpr double kClimbHomingSpeed = 1.0;
constexpr int kTicksPerMotorRotation = 42;

constexpr units::feet_per_second_t kClimberExtendSpeed = 1_fps;
constexpr double kClimberVelocityScalar = 1.0;

// Distance between left and right arm centers
constexpr units::meter_t kClimberOffsetDistance = 4_m;

static const SingleAxisMechanism kLeftClimberMechanism = {
    // min length
    24_in,
    // angle
    70_deg,
    // line width
    6.0,
    // color
    ColorConstants::kGreen};

static const SingleAxisMechanism kRightClimberMechanism = {
    // min length
    24_in,
    // angle
    70_deg,
    // line width
    6.0,
    // color
    ColorConstants::kRed};
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
  EnemyWing,
  AllianceWing,
};

typedef std::function<RobotState()> StateGetter;