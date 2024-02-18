#pragma once

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "commands/FeedCommand.h"
#include "commands/FlywheelRampCommand.h"
#include "commands/IntakeInInitialCommand.h"
#include "commands/IntakeInSecondaryCommand.h"
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
  // TODO: Shuffle the note down first and then feed it to the shooter via Feed
  // after ramping is done
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
      .Unless([intake] { return !intake->NotePresent(); })
      .WithTimeout(5_s)
      .FinallyDo([intake, scoring] {
        intake->Stop();
        scoring->Stop();
      });
}

}  // namespace ScoringCommands

namespace IntakingCommands {
using namespace IntakingConstants;

static frc2::CommandPtr Intake(IntakeSubsystem* intakeSubsystem) {
  return (frc2::InstantCommand([] {
            ConsoleLogger::getInstance().logVerbose("Intake Subsystem",
                                                    "Intake start %s", "");
          })
              .ToPtr()
              .AndThen(IntakeInInitial(intakeSubsystem).ToPtr())
              .AndThen(frc2::WaitCommand(0.2_s).ToPtr())
              .AndThen(IntakeInSecondary(intakeSubsystem).ToPtr())
              .AndThen(frc2::InstantCommand([] {
                         ConsoleLogger::getInstance().logVerbose(
                             "Intake Subsystem", "Intake finished itself%s",
                             "");
                       }).ToPtr()))

      // TODO: Run IntakeInSecondary for a bit longer after the note is detected
      // so that it lands in the right spot
      .Unless([intakeSubsystem] { return intakeSubsystem->NotePresent(); })
      .WithTimeout(5_s)
      .FinallyDo([intakeSubsystem] { intakeSubsystem->Stop(); });
}

}  // namespace IntakingCommands

namespace FunniCommands {
static frc2::CommandPtr FeedUntilNotPresent(IntakeSubsystem* intake,
                                            ScoringSubsystem* scoring,
                                            ScoringDirection direction) {
  return frc2::InstantCommand([] {
           ConsoleLogger::getInstance().logVerbose("Gamepiece Funni",
                                                   "Feeding to top%s", "");
         })
      .ToPtr()
      .AndThen(Feed(intake, scoring, [direction] { return direction; }).ToPtr())
      .Until([intake] { return !intake->NotePresentUpper(); })
      .FinallyDo([intake, scoring] {
        intake->Stop();
        scoring->Stop();
      });
}
static frc2::CommandPtr OuttakeUntilPresent(IntakeSubsystem* intake,
                                            ScoringSubsystem* scoring,
                                            ScoringDirection direction) {
  return frc2::InstantCommand([] {
           ConsoleLogger::getInstance().logVerbose("Gamepiece Funni",
                                                   "Outtaking down%s", "");
         })
      .ToPtr()
      .AndThen(frc2::InstantCommand([intake, scoring, direction] {
                 intake->Out(0.2);
                 scoring->SpinVectorSide(direction);
               }).ToPtr())
      .Until([intake] { return intake->NotePresentLower(); })
      .FinallyDo([intake, scoring] {
        intake->Stop();
        scoring->Stop();
      });
}

/*
Rainbow color

Cycle through:
- Feed to Amp side until top note not present
- Outtake until bottom note present
- Feed to Speaker side until top note not present
- Outtake until bottom note present
Unless note not present to begin with
Timeout of 20 seconds
Finally stop all motors
*/

static frc2::CommandPtr Funni(IntakeSubsystem* intake,
                              ScoringSubsystem* scoring, LedSubsystem* leds) {
  return (frc2::InstantCommand([] {
            ConsoleLogger::getInstance().logVerbose("Gamepiece Funni",
                                                    "Started%s", "");
          })
              .ToPtr()
              .AndThen(leds->Funni())
              .AndThen(FeedUntilNotPresent(intake, scoring,
                                           ScoringDirection::AmpSide))
              .AndThen(OuttakeUntilPresent(intake, scoring,
                                           ScoringDirection::SpeakerSide))
              .AndThen(FeedUntilNotPresent(intake, scoring,
                                           ScoringDirection::SpeakerSide))
              .AndThen(OuttakeUntilPresent(intake, scoring,
                                           ScoringDirection::AmpSide)))
      .Repeatedly()
      .Unless([intake] { return !intake->NotePresent(); })
      .WithTimeout(20_s)
      .FinallyDo([intake, scoring] {
        intake->Stop();
        scoring->Stop();
      });
}
}  // namespace FunniCommands