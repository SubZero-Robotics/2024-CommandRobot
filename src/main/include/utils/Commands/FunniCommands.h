#pragma once

#include <frc2/command/DeferredCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>

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
#include "utils/Commands/IntakeCommands.h"

namespace FunniCommands {
/*
Rainbow color

Cycle through:
- Feed to Amp side until top note not present
- Downtake until bottom note present
- Feed to Speaker side until top note not present
- Downtake until bottom note present
Unless note not present to begin with
Timeout of 20 seconds
Finally stop all motors
*/

static frc2::CommandPtr Funni(IntakeSubsystem* intake,
                              ScoringSubsystem* scoring, LedSubsystem* leds) {
  return (frc2::InstantCommand([] {
            ConsoleWriter.logVerbose("Gamepiece Funni",
                                                    "Started%s", "");
          })
              .ToPtr()
              .AndThen(leds->Funni())
              .AndThen(IntakingCommands::FeedUntilNotPresent(
                  intake, scoring, ScoringDirection::AmpSide))
              .AndThen(IntakingCommands::DowntakeUntilPresent(
                  intake, scoring, ScoringDirection::PodiumSide))
              .AndThen(IntakingCommands::FeedUntilNotPresent(
                  intake, scoring, ScoringDirection::PodiumSide))
              .AndThen(IntakingCommands::DowntakeUntilPresent(
                  intake, scoring, ScoringDirection::AmpSide)))
      .Repeatedly()
      .Unless([intake] { return !intake->NotePresent(); })
      .WithTimeout(20_s)
      .FinallyDo([intake, scoring] {
        intake->Stop();
        scoring->Stop();
      });
}
}  // namespace FunniCommands