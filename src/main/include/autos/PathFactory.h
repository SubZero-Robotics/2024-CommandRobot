#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/commands/FollowPathHolonomic.h>

#include <functional>

#include "Constants.h"
#include "OnTheFlyFactory.h"
#include "subsystems/DriveSubsystem.h"

using namespace AutoConstants::Locations;
using namespace pathplanner;

class PathFactory {
 public:
  static frc2::CommandPtr GetPathFromFinalLocation(
      std::function<FinalLocation()> locationGetter, DriveSubsystem* drive,
      frc2::CommandPtr&& prepCommand) {
    return GetApproxCommand(locationGetter())
        .AndThen(std::move(prepCommand))
        .AndThen(GetFinalApproachCommand(locationGetter(), drive));
  };

  static frc2::CommandPtr GetPathFromFinalLocation(
      std::function<FinalLocation()> locationGetter, DriveSubsystem* drive) {
    return GetApproxCommand(locationGetter())
        .AndThen(GetFinalApproachCommand(locationGetter(), drive));
  }

 private:
  static bool ShouldFlip(FinalLocation location) {
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
    ConsoleLogger::getInstance().logError("PPF", "Invalid Alliance%s", "");
    return false;
  }

  static frc2::CommandPtr GetApproxCommand(FinalLocation location) {
    auto& approxPose = OnTheFlyFactory::GetApproxLocation(location);
    frc::Pose2d betterapprox = approxPose;
    ConsoleLogger::getInstance().logInfo("PATH factory", betterapprox);
    return pathplanner::AutoBuilder::pathfindToPose(
        approxPose,
        pathplanner::PathConstraints{3.0_mps, 4.0_mps_sq, 540_deg_per_s,
                                     720_deg_per_s_sq},
        0.0_mps,  // Goal end velocity in meters/sec
        0.0_m     // Rotation delay distance in meters. This is how far
                  // the robot should travel before attempting to rotate.
    );
  }

  static frc2::CommandPtr GetFinalApproachCommand(FinalLocation location,
                                                  DriveSubsystem* drive) {
    auto path = PathPlannerPath::fromPathFile(PoseToPath.at(location));

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
                 return PathFactory::ShouldFlip(location);
               },
               {drive})
        .ToPtr();
  }
};