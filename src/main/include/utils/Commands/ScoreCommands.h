#pragma once

#include <frc2/command/DeferredCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>

#include <functional>

#include "Constants.h"
#include "commands/FeedCommand.h"
#include "commands/FlywheelRampCommand.h"
#include "commands/ShootCommand.h"
#include "commands/TurnToAngleCommand.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ScoringSubsystem.h"

static frc2::CommandPtr StopIntakeAndScoring(ScoringSubsystem* scoring,
                                             IntakeSubsystem* intake) {
  return frc2::FunctionalCommand(
             // onInit
             [] {},
             // onExecute
             [intake, scoring] {
               intake->Stop();
               scoring->Stop();
             },
             // onEnd
             [intake, scoring](bool interupted) {
               intake->Stop();
               scoring->Stop();
             },
             // isFinished
             [intake, scoring] { return true; },
             // req
             {intake, scoring})
      .ToPtr();
}

static frc2::CommandPtr RunUntilNotePresentUpper(ScoringSubsystem* scoring,
                                                 IntakeSubsystem* intake) {
  return ConsoleVerbose("Score Command", "RunUntilNotePresentUpper%s", "")
      .AndThen(frc2::FunctionalCommand(
                   // onInit
                   [] {},
                   // onExecute
                   [intake, scoring] {
                     intake->In();
                     scoring->SpinVectorSide(ScoringDirection::AmpSide);
                   },
                   // onEnd
                   [intake, scoring](bool interupted) {},
                   // isFinished
                   [intake, scoring] { return intake->NotePresentUpperAll(); },
                   // req
                   {intake, scoring})
                   .ToPtr())
      .AndThen(frc2::WaitCommand(0.02_s).ToPtr())
      .AndThen(StopIntakeAndScoring(scoring, intake));
}

static frc2::CommandPtr RunUntilNotePresentLower(ScoringSubsystem* scoring,
                                                 IntakeSubsystem* intake) {
  return ConsoleVerbose("Score Command", "RunUntilNotePresentLower%s", "")
      .AndThen(frc2::FunctionalCommand(
                   // onInit
                   [] {},
                   // onExecute
                   [intake, scoring] {
                     intake->Out();
                     scoring->SpinVectorSide(ScoringDirection::PodiumSide);
                   },
                   // onEnd
                   [intake, scoring](bool interupted) {},
                   // isFinished
                   [intake, scoring] { return intake->NotePresentLower(); },
                   // req
                   {intake, scoring})
                   .ToPtr())
      .AndThen(frc2::WaitCommand(0.02_s).ToPtr())
      .AndThen(StopIntakeAndScoring(scoring, intake));
}

static frc2::CommandPtr PreScoreShuffle(
    std::function<ScoringDirection()> direction, ScoringSubsystem* scoring,
    IntakeSubsystem* intake) {
  // This PreShuffle forces the note into the upper beam breaks before it is
  // pull back down

  return (ConsoleVerbose("Score Command", "Pre Score Shuffle%s", "")
              .AndThen(RunUntilNotePresentUpper(scoring, intake))
              .AndThen(RunUntilNotePresentLower(scoring, intake))
              .WithTimeout(1_s)
              .FinallyDo([intake, scoring] {
                intake->Stop();
                scoring->Stop();
              }));
}

namespace ScoringCommands {
using namespace ScoringConstants;

static frc2::CommandPtr ArmScoringGoal(
    std::function<ScoringDirection()> direction, ArmSubsystem* arm) {
  return frc2::DeferredCommand(
             [direction, arm] {
               if (direction() == ScoringDirection::AmpSide) {
                 return arm->MoveToPositionAbsolute(ArmConstants::kAmpRotation);
               } else {
                 // // In case it got stuck up, make it go down when not
                 // shooting
                 return frc2::InstantCommand([] {}).ToPtr();
               }
             },
             {arm})
      .ToPtr();
}

static frc2::CommandPtr ScoreRamp(std::function<ScoringDirection()> direction,
                                  ScoringSubsystem* scoring,
                                  IntakeSubsystem* intake, ArmSubsystem* arm) {
  return (ArmScoringGoal(direction, arm)
              .AndThen(FlywheelRamp(intake, scoring, direction)
                           .ToPtr()
                           .WithTimeout(2_s)
                           .AndThen(ConsoleVerbose("Scoring Composition",
                                                   "flywheel ramped%s", ""))))
      .Unless([intake] { return !intake->NotePresent(); })
      .WithTimeout(3_s);
}

static frc2::CommandPtr ScoreShoot(std::function<ScoringDirection()> direction,
                                   ScoringSubsystem* scoring,
                                   IntakeSubsystem* intake, ArmSubsystem* arm) {
  return (

             Feed(intake, scoring, direction)
                 .ToPtr()
                 .WithTimeout(2_s)
                 .AndThen(ConsoleVerbose("Scoring Composition", "fed%s", ""))
                 .AndThen(frc2::WaitCommand(kFlywheelRampDelay).ToPtr())
                 // .AndThen(PreScoreShuffle(direction, scoring, intake))
                 .AndThen(
                     Shoot(intake, scoring, direction).ToPtr().WithTimeout(1_s))
                 .AndThen(ConsoleVerbose("Scoring Composition", "shot%s", "")))
      .Unless([intake] { return !intake->NotePresent(); })
      .WithTimeout(3_s)
      .FinallyDo([intake, scoring] {
        intake->Stop();
        scoring->Stop();
      })
      .AndThen(frc2::DeferredCommand(
                   [direction, arm] {
                     if (direction() == ScoringDirection::AmpSide) {
                       return arm->MoveToPositionAbsolute(
                           ArmConstants::kHomeRotation);
                     } else {
                       return frc2::InstantCommand([] {}).ToPtr();
                     }
                   },
                   {arm})
                   .ToPtr());
}

static frc2::CommandPtr Score(std::function<ScoringDirection()> direction,
                              ScoringSubsystem* scoring,
                              IntakeSubsystem* intake, ArmSubsystem* arm) {
  return (
             // Downshuffle until either of the bottom two beam breaks is broken
             (PreScoreShuffle(direction, scoring, intake)
                  .AndThen(ScoreRamp(direction, scoring, intake, arm))
                  .AndThen(frc2::WaitCommand(kFlywheelRampDelay).ToPtr())
                  .AndThen(ScoreShoot(direction, scoring, intake, arm)
                               .FinallyDo([intake, scoring] {
                                 intake->Stop();
                                 scoring->Stop();
                               })))
                 .FinallyDo([intake, scoring] {
                   intake->Stop();
                   scoring->Stop();
                 })
             // Unless note isn't present
             // timeout after 3 seconds
             // finally stop everything
             )
      .Unless([intake] { return !intake->NotePresent(); })
      .WithTimeout(1.5_s)
      .FinallyDo([intake, scoring] {
        intake->Stop();
        scoring->Stop();
      });
}
}  // namespace ScoringCommands