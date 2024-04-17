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
#include "commands/IntakeInInitialCommand.h"
#include "commands/IntakeInSecondaryCommand.h"
#include "commands/NoteShuffle.h"
#include "commands/ShootCommand.h"
#include "commands/TurnToAngleCommand.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ScoringSubsystem.h"

namespace ScoringCommands {
using namespace ScoringConstants;

static frc2::CommandPtr OutakeUntilTopNotPresent(IntakeSubsystem* intake) {
  return (NoteShuffle(intake)
              .ToPtr()
              .AndThen(frc2::WaitCommand(0_s).ToPtr())
              .AndThen(
                  frc2::InstantCommand([intake] { intake->Stop(); }).ToPtr()))
      // .Unless([intake] { return !intake->NotePresent(); })
      .WithTimeout(5_s)
      .FinallyDo([intake] { intake->Stop(); });
}

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

static bool IsNoteTopSide(IntakeSubsystem* intake, ScoringDirection direction) {
  switch (direction) {
    case ScoringDirection::AmpSide:
    case ScoringDirection::Subwoofer:
      return intake->NotePresentUpperAmp();
    case ScoringDirection::PodiumSide:
      return intake->NotePresentUpperPodium();
  }
}

static frc2::CommandPtr PreScoreShuffle(
    std::function<ScoringDirection()> direction, ScoringSubsystem* scoring,
    IntakeSubsystem* intake) {
  return ConsoleInfo("Scoring", "Pre Score Shuffle%s", "")
      .AndThen(frc2::FunctionalCommand(
                   // on init
                   [scoring, intake, direction] {
                     scoring->SpinVectorSide(direction());
                     intake->Out();
                   },
                   // on execute
                   [] {},
                   // on end
                   [scoring, intake](bool interupted) {
                     scoring->Stop();
                     intake->Stop();
                   },
                   // is finished
                   [intake, direction] {
                     return !IsNoteTopSide(intake, direction());
                   },
                   // reqs
                   {intake, scoring})
                   .ToPtr())
      .WithTimeout(1_s)
      .FinallyDo([intake, scoring] {
        intake->Stop();
        scoring->Stop();
      });
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
             ScoreRamp(direction, scoring, intake, arm)
                 .AndThen(frc2::WaitCommand(kFlywheelRampDelay).ToPtr())
                 .AndThen(ScoreShoot(direction, scoring, intake, arm))
             // Unless note isn't present
             // timeout after 3 seconds
             // finally stop everything
             )
      .WithTimeout(3_s);
}
}  // namespace ScoringCommands