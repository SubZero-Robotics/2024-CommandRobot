#pragma once

#include <frc2/command/DeferredCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "ScoreCommands.h"
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
  return ConsoleVerbose("FeedUntilNotPresent", "Feeding to top%s", "")
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
  return (ConsoleVerbose("Downtake", "Downtake down %s", "start")
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
              .AndThen(ConsoleVerbose("Intake Subsystem",
                                      "Intake completed normally%s", "")))
      .WithTimeout(2_s)
      .FinallyDo([intake, scoring] {
        intake->Stop();
        scoring->Stop();
      });
}

static frc2::CommandPtr Intake(IntakeSubsystem* intake,
                               ScoringSubsystem* scoring) {
  return frc2::InstantCommand([intake, scoring] {
           scoring->SpinAmp(IntakingConstants::kPseudoBrakeModeSpeed,
                            IntakingConstants::kPseudoBrakeModeSpeed);
         })
      .ToPtr()
      .AndThen(RunUntilNotePresentUpper(scoring, intake))
      .AndThen(ConsoleInfo("Intk", "%s", "Intook"))
      .AndThen(frc2::WaitCommand(0.02_s).ToPtr())
      .AndThen(frc2::InstantCommand([intake, scoring] {
                 intake->Stop();
                 scoring->Stop();
               }).ToPtr())
      .AndThen(ConsoleInfo("Intk", "%s", "Stopped"))
      .AndThen(frc2::InstantCommand([intake] {
                 ConsoleWriter.logInfo("Intk", "Note present lower: %d",
                                       intake->NotePresentLower());
               }).ToPtr())
      .AndThen(frc2::FunctionalCommand(
                   // onInit
                   [] {},
                   // onExecute
                   [intake, scoring] {
                     intake->Out();
                     scoring->SpinVectorSide(ScoringDirection::PodiumSide);
                     scoring->SpinOutake(
                         ScoringConstants::kScoringIntakingOutakeUpperSpeed,
                         ScoringConstants::kScoringIntakingOutakeLowerSpeed);
                   },
                   // onEnd
                   [intake, scoring](bool interupted) {
                     intake->Stop();
                     scoring->Stop();
                   },
                   // isFinished
                   [intake, scoring] { return intake->NotePresentLower(); },
                   // req
                   {})
                   .ToPtr()
                   .HandleInterrupt([] {
                     ConsoleWriter.logInfo("Intk",
                                           "AAAA INTERRUPTED EARLY1!!11!1");
                   }))
      .AndThen(ConsoleInfo("Intk", "%s", "DownShuffled"))
      .HandleInterrupt(
          [] { ConsoleWriter.logInfo("Intk", "AAAA INTERRUPTED EARLY"); });
}

static frc2::CommandPtr Intake2(IntakeSubsystem* intake,
                                ScoringSubsystem* scoring) {
  return frc2::InstantCommand([intake, scoring] {
           //  intake->In(IntakingConstants::kIntakeAutoSpeed);
           intake->In();
           scoring->SpinVectorSide(ScoringDirection::AmpSide);
         })
      .ToPtr()
      .AndThen(frc2::WaitCommand(0.02_s).ToPtr())
      .AndThen(frc2::InstantCommand([intake, scoring] {
                 intake->Stop();
                 scoring->Stop();
               }).ToPtr())
      .AndThen(frc2::InstantCommand([intake, scoring] {
                 intake->Out();
                 scoring->SpinVectorSide(ScoringDirection::PodiumSide);
                 scoring->SpinOutake(
                     ScoringConstants::kScoringIntakingOutakeUpperSpeed,
                     ScoringConstants::kScoringIntakingOutakeLowerSpeed);
               }).ToPtr())
      .AndThen(frc2::InstantCommand([intake, scoring] {
                 intake->Stop();
                 scoring->Stop();
               }).ToPtr());
}

static frc2::CommandPtr IntakeAmpOnly(IntakeSubsystem* intake,
                                      ScoringSubsystem* scoring) {
  return frc2::FunctionalCommand(
             // onInit
             [] {},
             // onExecute
             [intake, scoring] {
               //  intake->In(IntakingConstants::kIntakeAutoSpeed);
               intake->In();
               scoring->SpinVectorSide(ScoringDirection::AmpSide);
               //  scoring->SpinOutake(
               //      ScoringConstants::kScoringIntakingOutakeUpperSpeed,
               //      ScoringConstants::kScoringIntakingOutakeLowerSpeed);
             },
             // onEnd
             [intake, scoring](bool interupted) {},
             // isFinished
             [intake, scoring] { return intake->NotePresentUpper(); },
             // req
             {intake, scoring})
      .ToPtr();
}
}  // namespace IntakingCommands