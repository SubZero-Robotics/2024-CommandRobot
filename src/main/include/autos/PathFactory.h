#pragma once

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
  static frc2::CommandPtr GetApproxCommand(FinalLocation location) {
    auto& approxPose = OnTheFlyFactory::GetApproxLocation(location);
    const auto betterapprox = approxPose;
    ConsoleLogger::getInstance().logInfo("PATH facrtory", &betterapprox);
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
               []() {
                 // todo, flip if is source
                 // reverse that if on red alliance
                 return false;
               },
               {drive})
        .ToPtr();
  }
};