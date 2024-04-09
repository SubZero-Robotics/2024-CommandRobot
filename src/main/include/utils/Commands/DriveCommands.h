#pragma once

#include <frc2/command/DeferredCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>

#include <algorithm>
#include <vector>

#include "Constants.h"
#include "commands/FeedCommand.h"
#include "commands/FlywheelRampCommand.h"
#include "commands/IntakeInInitialCommand.h"
#include "commands/IntakeInSecondaryCommand.h"
#include "commands/NoteShuffle.h"
#include "commands/ShootCommand.h"
#include "commands/TurnToAngleCommand.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ScoringSubsystem.h"

namespace DrivingCommands {
using namespace AutoConstants;
struct RelativeLocation {
  units::meter_t hypotDistance;
  frc::Pose2d trackedPose;
  frc::Pose2d fixtureLocation;
};

static std::vector<RelativeLocation> GetDistancesToFixtures(
    frc::Pose2d currentPose) {
  std::vector<RelativeLocation> locationDistances;
  auto alliance = frc::DriverStation::GetAlliance();
  if (!alliance) {
    return {};
  }
  auto side = alliance.value();
  auto& fixureLocations = side == frc::DriverStation::Alliance::kRed
                              ? Locations::RedFixtureLocations
                              : Locations::BlueFixtureLocations;
  locationDistances.reserve(fixureLocations.size());
  std::transform(fixureLocations.begin(), fixureLocations.end(),
                 std::back_inserter(locationDistances),
                 [currentPose](const Locations::FixtureLocation& loc) {
                   auto dif = currentPose - loc.fixtureLocation;
                   auto distance = std::hypot(dif.X().value(), dif.Y().value());
                   return RelativeLocation{
                       .hypotDistance = units::meter_t(distance),
                       .trackedPose = loc.trackedPose,
                       .fixtureLocation = loc.fixtureLocation};
                 });

  return locationDistances;
}

static RelativeLocation GetClosestFixture(
    std::vector<RelativeLocation>& locationDistances) {
  auto it = std::min_element(
      locationDistances.begin(), locationDistances.end(),
      [](const RelativeLocation& a, const RelativeLocation& b) {
        return a.hypotDistance < b.hypotDistance;
      });

  return *it;
}

static RelativeLocation LocationFromProximity(frc::Pose2d currentPose) {
  auto locationDistances = GetDistancesToFixtures(currentPose);

  return GetClosestFixture(locationDistances);
}

// static frc2::CommandPtr SnapToAngle(DriveSubsystem* drive) {
//   return (frc2::InstantCommand([] {
//             ConsoleLogger::getInstance().logInfo(
//                 "SnapToAngle", "Snapping to a new angle%s", "");
//           })
//               .ToPtr()
//               .AndThen(TurnToAngle(
//                            drive,
//                            [drive] {
//                              return RotationFromProximity(drive->GetPose());
//                            },
//                            false)
//                            .ToPtr()))
//       .WithTimeout(2_s)
//       .FinallyDo([drive] {
//         frc::ChassisSpeeds chassisSpeeds = {
//             .vx = 0_mps, .vy = 0_mps, .omega = 0_rad_per_s};
//         drive->Drive(chassisSpeeds);
//       });
// }
}  // namespace DrivingCommands