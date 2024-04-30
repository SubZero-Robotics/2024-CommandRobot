#include "subsystems/StateSubsystem.h"

#include "Constants.h"
#include "autos/PathFactory.h"
#include "commands/DriveVelocityCommand.h"
#include "commands/ExtendClimbCommand.h"
#include "commands/Funni.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "utils/ShuffleboardLogger.h"

using namespace AutoConstants::Locations;

StateSubsystem::StateSubsystem(Subsystems_t& subsystems,
                               frc2::CommandXboxController& driver,
                               frc2::CommandXboxController& op)
    : m_currentState(RobotState::Manual),
      m_subsystems{subsystems},
      m_driverController{driver},
      m_operatorController{op} {}

void StateSubsystem::IncrementState() {
  uint8_t nextState = (static_cast<uint8_t>(m_currentState)) + 1;
  m_currentState = static_cast<RobotState>(nextState % 6);
}

frc2::CommandPtr StateSubsystem::RunState() {
  ConsoleWriter.logVerbose("StateSubsystem", "Running with state %u",
                           static_cast<uint8_t>(m_currentState));

  switch (m_currentState) {
    case RobotState::Manual:
      return StartManual();
    case RobotState::ScoringSpeaker:
      return StartScoringSpeaker()
          .Until([this] {
            return IsControllerActive(m_driverController, m_operatorController);
          })
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::ScoringAmp:
      return StartScoringAmp()
          .Until([this] {
            return IsControllerActive(m_driverController, m_operatorController);
          })
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::ScoringSubwoofer:
      return StartScoringSubwoofer()
          .Until([this] {
            return IsControllerActive(m_driverController, m_operatorController);
          })
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::Intaking:
      return StartIntaking()
          .Until([this] {
            return IsControllerActive(m_driverController, m_operatorController);
          })
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::ClimbStageLeft:
    case RobotState::ClimbStageCenter:
    case RobotState::ClimbStageRight:
      return StartClimb()
          .Until([this] {
            return IsControllerActive(m_driverController, m_operatorController);
          })
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::SourceLeft:
    case RobotState::SourceCenter:
    case RobotState::SourceRight:
      return StartSource()
          .Until([this] {
            return IsControllerActive(m_driverController, m_operatorController);
          })
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::AutoSequenceAmp:
    case RobotState::AutoSequenceSpeaker:
    case RobotState::AutoSequenceSubwoofer:
      return StartAutoSequence()
          .Until([this] {
            return IsControllerActive(m_driverController, m_operatorController);
          })
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::Funni:
      return StartFunni()
          .Until([this] {
            return IsControllerActive(m_driverController, m_operatorController);
          })
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::AllianceWing:
      return StartAllianceWing()
          .Until([this] {
            return IsControllerActive(m_driverController, m_operatorController);
          })
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::EnemyWing:
      return StartEnemyWing()
          .Until([this] {
            return IsControllerActive(m_driverController, m_operatorController);
          })
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    default:
      return m_subsystems.led->Error()
          .AndThen(frc2::WaitCommand(1_s).ToPtr())
          .Until([this] {
            return IsControllerActive(m_driverController, m_operatorController);
          })
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
  }
}

frc2::CommandPtr StateSubsystem::StartIntaking() {
  return m_subsystems.led->ShowFromState([] { return RobotState::Intaking; })
      .AndThen(
          IntakingCommands::Intake(m_subsystems.intake, m_subsystems.scoring)
              .RaceWith(DriveVelocity(0_deg, 2_mps, m_subsystems.drive)
                            .ToPtr()
                            .Repeatedly())
              .Until([this] {
                return m_subsystems.intake->NotePresentUpper() &&
                       m_subsystems.intake->NotePresentLower();
              })
              .FinallyDo([this] {
                auto chassisSpeeds = frc::ChassisSpeeds::Discretize(
                    0_mps, 0_mps, 0_deg_per_s, DriveConstants::kLoopTime);
                m_subsystems.drive->Drive(chassisSpeeds);
              })
              // TODO: Put this in the "Loaded" state also have the intensity
              // vary based on if a note was successfully intooketh or not
              // .AndThen(ControllerCommands::Rumble(&m_driverController,
              //                                     [] { return 1_s; }))
              .WithTimeout(5_s));
}

frc2::CommandPtr StateSubsystem::StartScoringSpeaker() {
  /*
     Signal that we are scoring speaker
     Go to the speaker with PP on the fly path generation
     Orient the robot with the speaker shooter facing the speaker
     Shoot with timeout
  */

  return m_subsystems.led->ScoringSpeaker()
      .AndThen(PathFactory::GetPathFromFinalLocation(
          [] { return FinalLocation::Podium; }, m_subsystems.drive,
          m_subsystems.led))
      // ? TODO: Snap to angle first?
      .AndThen(ScoringCommands::Score(
          [] { return ScoringDirection::PodiumSide; }, m_subsystems.scoring,
          m_subsystems.intake, m_subsystems.arm))
      .WithTimeout(20_s);
}

frc2::CommandPtr StateSubsystem::StartScoringAmp() {
  /*
     Signal that we are scoring amp
     Go to the amp with PP on the fly path generation
     Orient the robot with the amp shooter facing the amp
     Shoot with timeout
  */
  return m_subsystems.led->ScoringAmp()
      .AndThen(PathFactory::GetPathFromFinalLocation(
          [] { return FinalLocation::Amp; }, m_subsystems.drive,
          m_subsystems.led))
      // ? TODO: Snap to angle first?
      .AndThen(ScoringCommands::Score([] { return ScoringDirection::AmpSide; },
                                      m_subsystems.scoring, m_subsystems.intake,
                                      m_subsystems.arm))
      .WithTimeout(20_s);
}

frc2::CommandPtr StateSubsystem::StartScoringSubwoofer() {
  /*
     Signal that we are scoring subwoofer
     Go to the speaker with PP on the fly path generation
     Orient the robot with the amp shooter facing the speaker
     Shoot with timeout
  */
  return m_subsystems.led->ScoringSubwoofer()
      .AndThen(PathFactory::GetPathFromFinalLocation(
          [] { return FinalLocation::Subwoofer; }, m_subsystems.drive,
          m_subsystems.led))
      // ? TODO: Snap to angle first?
      .AndThen(ScoringCommands::Score(
          [] { return ScoringDirection::Subwoofer; }, m_subsystems.scoring,
          m_subsystems.intake, m_subsystems.arm))
      .WithTimeout(20_s);
}

frc2::CommandPtr StateSubsystem::StartManual() {
  /*
     Signal that we are in manual
     (Call after automated commands or after manual intervention from driver 1)
  */
  return frc2::InstantCommand([this] {
           m_active = false;
           m_subsystems.led->IdlingAsync();
         })
      .ToPtr();
}

frc2::CommandPtr StateSubsystem::StartClimb() {
  /*
     Signal that we about to climb
     Use on the fly PP to go to approx. designated stage location
     Extend arms
     Use pre-made PP path to orient robot to chain and drive forward
     Retract arms
     Balance
  */

  return m_subsystems.led->Climbing()
      .AndThen(
          // TODO: method to get the stage location
          PathFactory::GetPathFromFinalLocation(
              [this] { return GetFinalFromState(); }, m_subsystems.drive,
              m_subsystems.led))
      .WithTimeout(20_s);
}

frc2::CommandPtr StateSubsystem::StartSource() {
  /*
     Signal that we about to go to the source
     Use on the fly PP to go to approx. location
     Use pre-made PP path to orient robot and drive forward
  */

  return m_subsystems.led->Outaking()
      .AndThen(PathFactory::GetPathFromFinalLocation(
          [this] { return GetFinalFromState(); }, m_subsystems.drive,
          m_subsystems.led))
      .WithTimeout(20_s);
}

frc2::CommandPtr StateSubsystem::StartAutoSequence() {
  /*
    Get the appropriate scoring location
    Chain commands:
      1. Auto-intake
      2. Score at location
      3. Go to source
  */

  return (MoveToSourceAndIntake()
              .AndThen(StartScoringAmp())
              .AndThen(MoveToSourceAndIntake())
              .AndThen(StartScoringAmp())
              .AndThen(MoveToSourceAndIntake())
              .AndThen(StartScoringSpeaker()))
      .Repeatedly();
}

frc2::CommandPtr StateSubsystem::StartFunni() {
  return FunniCommands::Funni(m_subsystems.intake, m_subsystems.scoring,
                              m_subsystems.led);
}

frc2::CommandPtr StateSubsystem::StartAllianceWing() {
  // TODO: signaling
  return PathFactory::PathfindApproximate([] { return FinalLocation::Scoring; })
      .WithTimeout(20_s);
}

frc2::CommandPtr StateSubsystem::StartEnemyWing() {
  // TODO: signaling
  return PathFactory::PathfindApproximate(
             [] { return FinalLocation::EnemyWing; })
      .WithTimeout(20_s);
}

frc2::CommandPtr StateSubsystem::MoveToSourceAndIntake() {
  return StartSource().AndThen(
      IntakingCommands::DowntakeUntilPresent(
          m_subsystems.intake, m_subsystems.scoring, ScoringDirection::AmpSide)
          // TODO: REMOVE THIS; ONLY FOR TESTING PURPOSES
          .WithTimeout(5_s));
}

bool StateSubsystem::IsControllerActive(
    frc2::CommandXboxController& driverController,
    frc2::CommandXboxController& operatorController) {
  bool active = false;
  int count = driverController.GetButtonCount();
  for (int i = 1; i <= count; i++) {
    active |= driverController.GetRawButton(i);
  }
  count = driverController.GetAxisCount();
  for (int i = 0; i < count; i++) {
    active |=
        abs(driverController.GetRawAxis(i)) >= OIConstants::kDriveDeadband;
  }
  count = driverController.GetPOVCount();
  for (int i = 0; i < count; i++) {
    active |= driverController.GetPOV(i) != -1;
  }
  // Check the operator's "stop" button
  active |= operatorController.Button(19).Get();
  if (active) {
    ConsoleWriter.logVerbose("StateSubsystem", "Controller interrupt! %s", "");
  }
  return active;
}