#pragma once

#include <frc/trajectory/TrapezoidProfile.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <map>
#include <string>
#include <vector>

#include "constants/ScoringConstants.h"
#include "utils/AutoChooser.h"

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 1.5_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

constexpr pathplanner::PathConstraints kMovementConstraints{
    3.0_mps, 1.5_mps_sq, 540_deg_per_s, 720_deg_per_s_sq};
constexpr pathplanner::PathConstraints kOnTheFlyPPConstraints{
    2.5_mps, 3_mps_sq, 540_deg_per_s, 720_deg_per_s_sq};
constexpr units::meters_per_second_t kOnTheFlyPPEndVelocity = 0_mps;
constexpr units::meter_t kOnTheFlyRotationDelay = 0_m;

enum class AutoType {
  EmptyAuto = 0,
  FourNoteAuto,
  PlaceAndLeave,
  ThreeNoteAuto,
  TwoNoteAmpSide,
  LeaveWing,
  TwoNoteCenter,
  TwoNoteSource,
  ThreeNoteCenter,
  TwoNoteAuto,
  TwoNoteInAmp,
  TwoInSpeakerTwoInAmp,
};

static const std::vector<AutoChooser<AutoType>::AutoChooserEntry>
    kChooserEntries{
        {{AutoType::FourNoteAuto, "4 Note Auto"},
         {"4 notes", "close", "high", "amp", "center"}},
        {{AutoType::ThreeNoteAuto, "3 Note Auto"},
         {"3 notes", "close", "decent", "center"}},
        {{AutoType::TwoNoteCenter, "2 Note Center Note Under Stage"},
         {"2 notes", "far", "high", "center"}},
        {{AutoType::LeaveWing, "Leave Wing"},
         {"0 notes", "minimal", "close", "source"}},
        {{AutoType::PlaceAndLeave, "Place and leave"},
         {"1 note", "minimal", "mid", "source"}},
        {{AutoType::TwoNoteAmpSide, "2 Note Auto"},
         {"2 notes", "decent", "far", "amp"}},
        {{AutoType::TwoNoteSource, "2 Note Source Side"},
         {"2 notes", "decent", "far", "source"}},
        {{AutoType::ThreeNoteCenter, "3 Note Center Note 3 + 4"},
         {"3 notes", "high", "far", "source", "center"}},
        {{AutoType::EmptyAuto, "Empty Auto"},
         {"0 notes", "minimal", "close", "center"}},
        {{AutoType::TwoNoteAuto, "2 Note Auto"},
         {"2 notes", "decent", "far", "amp"}},
    };

static const std::vector<AutoChooser<AutoType>::AutoChooserSelectorGroup>
    kChooserGroups{
        {"Note Count", {"0 notes", "1 note", "2 notes", "3 notes", "4 notes"}},
        {"Proximity", {"close", "mid", "far"}},
        {"Impact", {"minimal", "decent", "high"}},
        {"Motion", {"amp", "center", "source", "complex"}},
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
constexpr frc::Pose2d ApproxAllianceWingLocation =
    frc::Pose2d{2.82_m, 6.07_m, 0_deg};
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
  // The distance around the fixtureLocation where scoring should occur
  std::optional<units::inch_t> scoringRadius;
  // How to shoot the note when in range of scoringRadius; defaults to Amp
  std::optional<ScoringDirection> scoringDirection;
  // What the robot should "look" at
  frc::Pose2d trackedPose;
};

const std::vector<FixtureLocation> RedFixtureLocations{
    // Podium (ScoreSpeaker)
    {.fixtureLocation = frc::Pose2d(13.8_m, 4.12_m, frc::Rotation2d(0_deg)),
     .locationRadius = 3_ft,
     .scoringRadius = 1.5_ft,
     .scoringDirection = ScoringDirection::PodiumSide,
     .trackedPose = frc::Pose2d(16.54_m, 5.6_m, frc::Rotation2d(0_deg))},
    // Amp (ScoreAmp)
    {.fixtureLocation = frc::Pose2d(14.64_m, 8.5_m, frc::Rotation2d(0_deg)),
     .locationRadius = 6_ft,
     .trackedPose = frc::Pose2d(14.75_m, 20_m, frc::Rotation2d(180_deg))},
    // Speaker (ScoreSubwoofer)
    {.fixtureLocation = frc::Pose2d(15.34_m, 5.6_m, frc::Rotation2d(0_deg)),
     .locationRadius = 6_ft,
     .scoringRadius = 4_ft,
     .scoringDirection = ScoringDirection::Subwoofer,
     .trackedPose = frc::Pose2d(16.54_m, 5.6_m, frc::Rotation2d(180_deg))},
    // Feeding (Feed)
    {.fixtureLocation = frc::Pose2d(7_m, 1.5_m, frc::Rotation2d(0_deg)),
     // TODO: bigger radius and motor velocity changes based on distance
     .locationRadius = 3_ft,
     .scoringRadius = 3_ft,
     .scoringDirection = ScoringDirection::FeedPodium,
     .trackedPose = frc::Pose2d(16_m, 7_m, frc::Rotation2d(0_deg))}};
const std::vector<FixtureLocation> BlueFixtureLocations{
    // Podium (ScoreSpeaker)
    // TODO
    {.fixtureLocation = frc::Pose2d(2.9_m, 4.15_m, frc::Rotation2d(0_deg)),
     .locationRadius = 3_ft,
     .scoringRadius = 1.5_ft,
     .scoringDirection = ScoringDirection::PodiumSide,
     .trackedPose = frc::Pose2d(0_m, 5.6_m, frc::Rotation2d(0_deg))},
    // Amp (ScoreAmp)
    {.fixtureLocation = frc::Pose2d(1.82_m, 8.5_m, frc::Rotation2d(0_deg)),
     .locationRadius = 6_ft,
     .trackedPose = frc::Pose2d(1.82_m, 20_m, frc::Rotation2d(180_deg))},
    // Speaker (ScoreSubwoofer)
    {.fixtureLocation = frc::Pose2d(0.5_m, 5.6_m, frc::Rotation2d(0_deg)),
     .locationRadius = 6_ft,
     .scoringRadius = 4_ft,
     .scoringDirection = ScoringDirection::Subwoofer,
     .trackedPose = frc::Pose2d(0_m, 5.6_m, frc::Rotation2d(180_deg))},
    // Feeding (Feed)
    {.fixtureLocation = frc::Pose2d(10_m, 1.5_m, frc::Rotation2d(0_deg)),
     // TODO: bigger radius and motor velocity changes based on distance
     .locationRadius = 3_ft,
     .scoringRadius = 3_ft,
     .scoringDirection = ScoringDirection::FeedPodium,
     .trackedPose = frc::Pose2d(1_m, 7_m, frc::Rotation2d(0_deg))}};
}  // namespace Locations
}  // namespace AutoConstants