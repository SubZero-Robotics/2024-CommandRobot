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
#include "commands/TurnToAngleCommand.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ScoringSubsystem.h"

namespace ControllerCommands {

// static frc2::CommandPtr Rumble(frc2::CommandXboxController* controller,
//                                std::function<units::second_t()> timeout) {
//   return frc2::InstantCommand([controller] {
//            controller->SetRumble(frc::GenericHID::RumbleType::kBothRumble,
//                                  OIConstants::kVibrationIntensity);
//          })
//       .ToPtr()
//       .AndThen(frc2::WaitCommand(timeout()).ToPtr())
//       .AndThen(frc2::InstantCommand([controller] {
//                  controller->SetRumble(frc::GenericHID::RumbleType::kBothRumble,
//                                        0);
//                }).ToPtr());
// }

}  // namespace ControllerCommands

namespace ScoringCommands {
using namespace ScoringConstants;

static frc2::CommandPtr OutakeUntilTopNotPresent(IntakeSubsystem* intake) {
  return (frc2::InstantCommand([intake] { intake->Out(); })
              .ToPtr()
              .Until([intake] { return !intake->NotePresentUpper(); })
              .AndThen(frc2::WaitCommand(0_s).ToPtr())
              .AndThen(
                  frc2::InstantCommand([intake] { intake->Stop(); }).ToPtr()))
      .Unless([intake] { return !intake->NotePresent(); })
      .WithTimeout(5_s)
      .FinallyDo([intake] { intake->Stop(); });
}

static frc2::CommandPtr Score(std::function<ScoringDirection()> direction,
                              ScoringSubsystem* scoring,
                              IntakeSubsystem* intake) {
  return (FlywheelRamp(intake, scoring, direction)
              .ToPtr()
              // .AlongWith(OutakeUntilTopNotPresent(intake))
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
              .AndThen(frc2::WaitCommand(0.1_s).ToPtr())
              .AndThen(IntakeInSecondary(intakeSubsystem).ToPtr())
              // .AndThen(frc2::WaitCommand(0_s).ToPtr())
              .AndThen(frc2::InstantCommand([intakeSubsystem] {
                         intakeSubsystem->Stop();
                       }).ToPtr())
              .AndThen(frc2::InstantCommand([] {
                         ConsoleLogger::getInstance().logVerbose(
                             "Intake Subsystem", "Intake finished itself%s",
                             "");
                       }).ToPtr()))
      .Unless([intakeSubsystem] { return intakeSubsystem->NotePresent(); })
      .WithTimeout(5_s)
      .FinallyDo([intakeSubsystem] { intakeSubsystem->Stop(); });
}

}  // namespace IntakingCommands

namespace DrivingCommands {
using namespace AutoConstants;
struct RelativeLocation {
  units::meter_t hypotDistance;
  units::degree_t desiredRotation;
};

static units::degree_t RotationFromProximity(DriveSubsystem* drive) {
  auto currentPose = drive->GetPose();
  std::vector<RelativeLocation> locationDistances;
  auto alliance = frc::DriverStation::GetAlliance();
  if (!alliance) {
    return 0_deg;
  }
  auto side = alliance.value();
  auto& fixureLocations = side == frc::DriverStation::Alliance::kRed
                              ? Locations::RedFixtureLocations
                              : Locations::BlueFixtureLocations;
  locationDistances.reserve(fixureLocations.size());
  std::transform(fixureLocations.begin(), fixureLocations.end(),
                 std::back_inserter(locationDistances),
                 [currentPose](const Locations::FixtureLocation& loc) {
                   auto dif = currentPose - loc.location;
                   auto distance = std::hypot(dif.X().value(), dif.Y().value());
                   return RelativeLocation{
                       .hypotDistance = units::meter_t(distance),
                       .desiredRotation = loc.desiredRotation};
                 });
  auto it = std::min_element(
      locationDistances.begin(), locationDistances.end(),
      [](const RelativeLocation& a, const RelativeLocation& b) {
        return a.hypotDistance < b.hypotDistance;
      });
  return it->desiredRotation;
}

static frc2::CommandPtr SnapToAngle(DriveSubsystem* drive) {
  return (frc2::InstantCommand([] {
            ConsoleLogger::getInstance().logInfo(
                "SnapToAngle", "Snapping to a new angle%s", "");
          })
              .ToPtr()
              .AndThen(TurnToAngle(
                           drive,
                           [drive] { return RotationFromProximity(drive); },
                           false)
                           .ToPtr()))
      .WithTimeout(2_s)
      .FinallyDo([drive] {
        frc::ChassisSpeeds chassisSpeeds = {
            .vx = 0_mps, .vy = 0_mps, .omega = 0_rad_per_s};
        drive->Drive(chassisSpeeds);
      });
}
}  // namespace DrivingCommands

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