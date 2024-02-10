#pragma once

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "commands/FeedCommand.h"
#include "commands/FlywheelRampCommand.h"
#include "commands/ShootCommand.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ScoringSubsystem.h"

namespace ControllerCommands {

static frc2::CommandPtr Rumble(frc2::CommandXboxController* controller,
                               std::function<units::second_t()> timeout) {
  return frc2::InstantCommand([controller] {
           controller->SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                 OIConstants::kVibrationIntensity);
         })
      .ToPtr()
      .AndThen(frc2::WaitCommand(timeout()).ToPtr())
      .AndThen(frc2::InstantCommand([controller] {
                 controller->SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                       0);
               }).ToPtr());
}

}  // namespace ControllerCommands

namespace ScoringCommands {
using namespace ScoringConstants;

static frc2::CommandPtr Score(std::function<ScoringDirection()> direction,
                              ScoringSubsystem* scoring,
                              IntakeSubsystem* intake) {
  return (FlywheelRamp(intake, scoring, direction)
              .ToPtr()
              .AndThen(frc2::InstantCommand([] {
                         ConsoleLogger::getInstance().logVerbose("Next",
                                                                 "next %s", "");
                       }).ToPtr())
              .AndThen(frc2::WaitCommand(kFlywheelRampDelay).ToPtr())
              .AndThen(Feed(intake, scoring, direction).ToPtr())
              .AndThen(frc2::WaitCommand(kFlywheelRampDelay).ToPtr())
              .AndThen(Shoot(intake, scoring, direction).ToPtr()))
      .WithTimeout(5_s)
      .FinallyDo([intake, scoring] {
        intake->Stop();
        scoring->Stop();
      });
}

}  // namespace ScoringCommands