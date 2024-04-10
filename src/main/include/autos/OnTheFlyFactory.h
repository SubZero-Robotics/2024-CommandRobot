#pragma once

#include <frc/geometry/Pose2d.h>

#include "Constants.h"

class OnTheFlyFactory {
 public:
  // Method to take in a final location
  // and return a approx location Pose2d
  static const frc::Pose2d& GetApproxLocation(
      AutoConstants::Locations::FinalLocation location) {
    using namespace AutoConstants::Locations;

    switch (location) {
      case FinalLocation::Subwoofer:
      case FinalLocation::Amp:
      case FinalLocation::Podium:
      case FinalLocation::StageLeft:
        return ApproxScoringLocation;

      case FinalLocation::Source1:
      case FinalLocation::Source2:
      case FinalLocation::Source3:
      case FinalLocation::StageRight:
        return ApproxSourceLocation;

      case FinalLocation::CenterStage:
        return ApproxCentralLocation;

      case FinalLocation::Scoring:
        return ApproxAllianceWingLocation;

      case FinalLocation::EnemyWing:
        return ApproxEnemyWingLocation;

      default:
        // TODO: Make this smarter
        return ApproxScoringLocation;
    }
  }
};