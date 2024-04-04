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

namespace IntakingCommands {
using namespace IntakingConstants;

static frc2::CommandPtr FeedUntilNotPresent(IntakeSubsystem* intake,
                                            ScoringSubsystem* scoring,
                                            ScoringDirection direction) {
  return frc2::InstantCommand([] {
           ConsoleLogger::getInstance().logVerbose("FeedUntilNotPresent",
                                                   "Feeding to top%s", "");
         })
      .ToPtr()
      .AndThen(Feed(intake, scoring, [direction] { return direction; })
                   .ToPtr()
                   .Until([intake] { return intake->NotePresentUpper(); })
                   .WithTimeout(2_s))
      // Might need to be time-based instead
      .WithTimeout(4_s);
}

static frc2::CommandPtr DowntakeUntilPresent(IntakeSubsystem* intake,
                                             ScoringSubsystem* scoring,
                                             ScoringDirection direction) {
  return (frc2::InstantCommand([] {
            ConsoleLogger::getInstance().logVerbose("Downtake",
                                                    "Downtake down%s", "");
          })
              .ToPtr()
              .AndThen(
                  frc2::InstantCommand([intake, scoring, direction] {
                    intake->Out(kDowntakeSpeed);
                    scoring->SpinVectorSide(direction);
                    scoring->SpinOutake();
                  })
                      .ToPtr()
                      .Repeatedly()
                      .Until([intake] { return intake->NotePresentLower(); })
                      .FinallyDo([intake, scoring] {
                        intake->Stop();
                        scoring->Stop();
                      }))
              .AndThen(FeedUntilNotPresent(intake, scoring,
                                           ScoringDirection::AmpSide))
              .AndThen(frc2::WaitCommand(0.1_s).ToPtr())
              .AndThen(frc2::InstantCommand([intake, scoring] {
                         intake->Stop();
                         scoring->Stop();
                       }).ToPtr())
              .AndThen(frc2::InstantCommand([] {
                         ConsoleLogger::getInstance().logVerbose(
                             "Intake Subsystem", "Intake completed normally%s",
                             "");
                       }).ToPtr()))
      .WithTimeout(2_s)
      .FinallyDo([intake, scoring] {
        intake->Stop();
        scoring->Stop();
      });
}

static frc2::CommandPtr Intake(IntakeSubsystem* intakeSubsystem,
                               ScoringSubsystem* scoring) {
  return (frc2::InstantCommand([] {
            ConsoleLogger::getInstance().logVerbose("Intake Subsystem",
                                                    "Intake start %s", "");
          })
              .ToPtr()
              .AndThen(IntakeInInitial(intakeSubsystem).ToPtr())
              .AndThen(frc2::WaitCommand(0.05_s).ToPtr())
              .AndThen(FeedUntilNotPresent(intakeSubsystem, scoring,
                                           ScoringDirection::AmpSide))
              .AndThen(frc2::WaitCommand(0.2_s).ToPtr())
              .AndThen(frc2::InstantCommand([intakeSubsystem, scoring] {
                         intakeSubsystem->Stop();
                         scoring->Stop();
                       }).ToPtr())
              .AndThen(frc2::InstantCommand([intakeSubsystem, scoring] {
                         intakeSubsystem->Stop();
                         scoring->Stop();
                       }).ToPtr())
              .AndThen(DowntakeUntilPresent(intakeSubsystem, scoring,
                                            ScoringDirection::PodiumSide))
              .AndThen(frc2::InstantCommand([] {
                         ConsoleLogger::getInstance().logVerbose(
                             "Intake Subsystem", "Intake completed normally%s",
                             "");
                       }).ToPtr()))
      .Unless([intakeSubsystem] { return intakeSubsystem->NotePresent(); })
      .WithTimeout(8_s)
      .FinallyDo([intakeSubsystem, scoring] {
        intakeSubsystem->Stop();
        scoring->Stop();
      });
}

static frc2::CommandPtr UnbiasNote(IntakeSubsystem* intake,
                                   ScoringSubsystem* scoring) {
  return frc2::FunctionalCommand(
             [] {},
             [scoring] {
               scoring->SpinVectorSide(ScoringDirection::PodiumSide);
             },
             [scoring](bool interruped) { scoring->Stop(); },
             [intake] { return intake->NotePresentLower(); }, {intake, scoring})
      .ToPtr();
}

static frc2::CommandPtr Intake2(IntakeSubsystem* intakeSubsystem,
                                ScoringSubsystem* scoring) {
  // Intake until top is broken biasing amp side
  // stop intake
  // downshuffle until either of the lower beam breaks is broken
  // stop intake
  // spin vector wheel to unbias the note
  // stop vector wheel
  // unless note is already present
  // timeout after 5 seconds
  // finally stop everything just in case something got canceled early

  return (IntakeInInitial(intakeSubsystem)
              .ToPtr()
              .AlongWith(frc2::InstantCommand([scoring] {
                           scoring->SpinVectorSide(ScoringDirection::AmpSide);
                         }).ToPtr())
              .WithTimeout(8_s)
              .AndThen(frc2::InstantCommand([intakeSubsystem, scoring] {
                         intakeSubsystem->Stop();
                         scoring->Stop();
                       }).ToPtr())
              .AndThen(frc2::InstantCommand([intakeSubsystem, scoring] {
                         intakeSubsystem->Out();
                         scoring->SpinVectorSide(ScoringDirection::PodiumSide);
                       }).ToPtr())
              .AndThen(frc2::WaitCommand(0.05_s).ToPtr())
              .AndThen(frc2::InstantCommand([intakeSubsystem, scoring] {
                         intakeSubsystem->Stop();
                         scoring->Stop();
                       }).ToPtr()))
      .Unless([intakeSubsystem] { return intakeSubsystem->NotePresent(); })
      .WithTimeout(8_s)
      .FinallyDo([intakeSubsystem, scoring] {
        intakeSubsystem->Stop();
        scoring->Stop();
      });
}

static frc2::CommandPtr Intake3(IntakeSubsystem* intake,
                                ScoringSubsystem* scoring) {
  return frc2::FunctionalCommand(
             // onInit
             [] {},
             // onExecute
             [intake, scoring] {
               //  intake->In(IntakingConstants::kIntakeAutoSpeed);
               intake->In();
               scoring->SpinVectorSide(ScoringDirection::AmpSide);
             },
             // onEnd
             [intake, scoring](bool interupted) {
               intake->Stop();
               scoring->Stop();
             },
             // isFinished
             [intake, scoring] { return intake->NotePresentUpperCenter(); },
             // req
             {intake, scoring})
      .ToPtr()
      .AndThen(frc2::FunctionalCommand(
                   // onInit
                   [] {},
                   // onExecute
                   [intake, scoring] {
                     intake->Out();
                     scoring->SpinVectorSide(ScoringDirection::PodiumSide);
                   },
                   // onEnd
                   [intake, scoring](bool interupted) {
                     intake->Stop();
                     scoring->Stop();
                   },
                   // isFinished
                   [intake, scoring] { return intake->NotePresentLower(); },
                   // req
                   {intake, scoring})
                   .ToPtr());
}
}