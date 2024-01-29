#pragma once

#include <frc/geometry/Pose2d.h>

#include "Constants.h"

using namespace AutoConstants::Locations;

class OnTheFlyFactory {
 public:
  // Method to take in a final location
  // and return a approx location Pose2d
  static const frc::Pose2d& GetApproxLocation(FinalLocation location) {
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
      default:
        // TODO: Make this smarter
        return ApproxScoringLocation;
    }
  }
};