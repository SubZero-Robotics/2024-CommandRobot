#pragma once

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"

namespace ControllerCommands {

static frc2::CommandPtr Rumble(frc2::CommandXboxController* controller,
                               std::function<units::second_t()> timeout) {
  return frc2::InstantCommand([controller] {
           controller->SetRumble(frc::GenericHID::RumbleType::kBothRumble, OIConstants::kVibrationIntensity);
         })
      .ToPtr()
      .AndThen(frc2::WaitCommand(timeout()).ToPtr())
      .AndThen(frc2::InstantCommand([controller] {
                 controller->SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                       0);
               }).ToPtr());
};

}  // namespace ControllerCommands