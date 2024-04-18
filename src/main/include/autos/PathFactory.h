#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/commands/FollowPathHolonomic.h>

#include <functional>
#include <utility>

#include "Constants.h"
#include "OnTheFlyFactory.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LedSubsystem.h"

class PathFactory {
 public:
  static frc2::CommandPtr GetPathFromFinalLocation(
      std::function<AutoConstants::Locations::FinalLocation()> locationGetter,
      DriveSubsystem* drive, LedSubsystem* leds,
      frc2::CommandPtr&& prepCommand) {
    return GetApproxCommand(locationGetter())
        .AndThen(std::move(prepCommand))
        .AndThen(GetFinalApproachCommand(locationGetter(), drive));
  }

  static frc2::CommandPtr GetPathFromFinalLocation(
      std::function<AutoConstants::Locations::FinalLocation()> locationGetter,
      DriveSubsystem* drive, LedSubsystem* leds) {
    return leds->OnTheFlyPP()
        .AndThen(GetApproxCommand(locationGetter())
                     .AndThen(GetFinalApproachCommand(locationGetter(), drive)))
        .AndThen(leds->Idling());
  }

  static frc2::CommandPtr PathfindApproximate(
      std::function<AutoConstants::Locations::FinalLocation()> locationGetter) {
    return GetApproxCommand(locationGetter());
  }

 private:
  static bool ShouldFlip(AutoConstants::Locations::FinalLocation location) {
    using namespace AutoConstants::Locations;

    auto alliance = frc::DriverStation::GetAlliance();
    if (!alliance) {
      return false;
    }
    auto side = alliance.value();

    if (side == frc::DriverStation::kBlue) {
      if (location == FinalLocation::Source1 ||
          location == FinalLocation::Source2 ||
          location == FinalLocation::Source3) {
        return true;
      }
      return false;
    }
    if (side == frc::DriverStation::kRed) {
      if (location == FinalLocation::Source1 ||
          location == FinalLocation::Source2 ||
          location == FinalLocation::Source3) {
        return false;
      }
      return true;
    }
    ConsoleWriter.logError("PPF", "Invalid Alliance%s", "");
    return false;
  }

  static frc2::CommandPtr GetApproxCommand(
      AutoConstants::Locations::FinalLocation location) {
    auto& approxPose = OnTheFlyFactory::GetApproxLocation(location);
    frc::Pose2d betterapprox = approxPose;
    ConsoleWriter.logInfo("PATH factory", betterapprox);
    if (ShouldFlip(location)) {
      return pathplanner::AutoBuilder::pathfindToPoseFlipped(
          approxPose,
          pathplanner::PathConstraints{2.5_mps, 3_mps_sq, 540_deg_per_s,
                                       720_deg_per_s_sq},
          0.0_mps,  // Goal end velocity in meters/sec
          0.0_m     // Rotation delay distance in meters. This is how far
                    // the robot should travel before attempting to rotate.
      );
    }

    return pathplanner::AutoBuilder::pathfindToPose(
        approxPose,
        pathplanner::PathConstraints{2.5_mps, 3_mps_sq, 540_deg_per_s,
                                     720_deg_per_s_sq},
        0.0_mps,  // Goal end velocity in meters/sec
        0.0_m     // Rotation delay distance in meters. This is how far
                  // the robot should travel before attempting to rotate.
    );
  }

  static frc2::CommandPtr GetFinalApproachCommand(
      AutoConstants::Locations::FinalLocation location, DriveSubsystem* drive) {
    using namespace pathplanner;

    auto path = PathPlannerPath::fromPathFile(
        AutoConstants::Locations::PoseToPath.at(location));

    return FollowPathHolonomic(
               path, [drive]() { return drive->GetPose(); },
               [drive]() { return drive->getSpeed(); },
               [drive](frc::ChassisSpeeds speeds) -> void {
                 drive->Drive(speeds);
               },
               AutoConstants::PathConfig,
               [location]() {
                 // todo, flip if is source
                 // reverse that if on red alliance
                 return ShouldFlip(location);
               },
               {drive})
        .ToPtr();
  }
};