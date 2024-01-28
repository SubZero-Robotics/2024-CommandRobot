#pragma once

#include "subsystems/DriveSubsystem.h"
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/commands/FollowPathHolonomic.h>

using namespace AutoConstants::Locations;
using namespace pathplanner;

class PathFactory {
  // Method to take in a final location
  // and return a path name
  frc2::CommandPtr GetPathFromFinalLocation(FinalLocation location, DriveSubsystem *drive) {
    auto path = PathPlannerPath::fromPathFile(PoseToPath.at(location));

    return FollowPathHolonomic(
        path,
        [drive](){ return drive->GetPose();},
        [drive](){ return drive->getSpeed();},
        [drive](frc::ChassisSpeeds speeds) -> void { drive->Drive(speeds); },
        AutoConstants::PathConfig,
        []() {
            // todo, flip if is source
            // reverse that if on red alliance
            return false;
        },
        { drive }
    ).ToPtr();
  };
};