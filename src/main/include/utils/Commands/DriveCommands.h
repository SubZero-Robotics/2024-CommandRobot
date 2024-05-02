#pragma once

#include <frc/DriverStation.h>
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
  units::meter_t locationRadius;
  std::optional<units::meter_t> scoringRadius;
  std::optional<ScoringDirection> scoringDirection;
};

static std::vector<Locations::FixtureLocation> GetFixtureLocations() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (!alliance) {
    return {};
  }
  auto side = alliance.value();
  auto& fixtureLocations = side == frc::DriverStation::Alliance::kRed
                               ? Locations::RedFixtureLocations
                               : Locations::BlueFixtureLocations;
  return fixtureLocations;
}

static std::vector<RelativeLocation> GetDistancesToFixtures(
    frc::Pose2d currentPose) {
  std::vector<RelativeLocation> locationDistances;
  auto fixtureLocations = GetFixtureLocations();
  locationDistances.reserve(fixtureLocations.size());
  std::transform(fixtureLocations.begin(), fixtureLocations.end(),
                 std::back_inserter(locationDistances),
                 [currentPose](const Locations::FixtureLocation& loc) {
                   auto dif = currentPose - loc.fixtureLocation;
                   auto distance = std::hypot(dif.X().value(), dif.Y().value());
                   return RelativeLocation{
                       .hypotDistance = units::meter_t(distance),
                       .trackedPose = loc.trackedPose,
                       .fixtureLocation = loc.fixtureLocation,
                       .locationRadius = loc.locationRadius,
                       .scoringRadius = loc.scoringRadius,
                       .scoringDirection = loc.scoringDirection};
                 });

  return locationDistances;
}

static std::vector<RelativeLocation> GetSortedLocations(
    frc::Pose2d currentPose) {
  auto locationDistances = GetDistancesToFixtures(currentPose);

  std::sort(locationDistances.begin(), locationDistances.end(),
            [](const RelativeLocation& a, const RelativeLocation& b) {
              return a.hypotDistance < b.hypotDistance;
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

  auto closestFixture = GetClosestFixture(locationDistances);
  return closestFixture;
}
}  // namespace DrivingCommands