#include "subsystems/StateSubsystem.h"

#include "Constants.h"
#include "autos/PathFactory.h"
#include "commands/BalanceCommand.h"
#include "commands/DriveVelocityCommand.h"
#include "commands/ExtendAbsoluteCommand.h"
#include "commands/ExtendClimbCommand.h"
#include "commands/Funni.h"
#include "commands/IntakeInCommand.h"
#include "commands/IntakeOutCommand.h"
#include "commands/RetractClimbCommand.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "utils/CommandUtils.h"
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
  uint8_t nextState = ((uint8_t)m_currentState) + 1;
  m_currentState = (RobotState)(nextState % 6);
}

frc2::CommandPtr StateSubsystem::RunState() {
  ConsoleLogger::getInstance().logVerbose(
      "StateSubsystem", "Running with state %u", (uint8_t)m_currentState);

  switch (m_currentState) {
    case RobotState::Manual:
      return StartManual();
    case RobotState::ScoringSpeaker:
      return StartScoringSpeaker()
          .Until(std::bind(&StateSubsystem::IsControllerActive, this))
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::ScoringAmp:
      return StartScoringAmp()
          .Until(std::bind(&StateSubsystem::IsControllerActive, this))
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::ScoringSubwoofer:
      return StartScoringSubwoofer()
          .Until(std::bind(&StateSubsystem::IsControllerActive, this))
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::Intaking:
      return StartIntaking()
          .Until(std::bind(&StateSubsystem::IsControllerActive, this))
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::ClimbStageLeft:
    case RobotState::ClimbStageCenter:
    case RobotState::ClimbStageRight:
      return StartClimb()
          .Until(std::bind(&StateSubsystem::IsControllerActive, this))
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::SourceLeft:
    case RobotState::SourceCenter:
    case RobotState::SourceRight:
      return StartSource()
          .Until(std::bind(&StateSubsystem::IsControllerActive, this))
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::AutoSequenceAmp:
    case RobotState::AutoSequenceSpeaker:
    case RobotState::AutoSequenceSubwoofer:
      return StartAutoSequence()
          .Until(std::bind(&StateSubsystem::IsControllerActive, this))
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    case RobotState::Funni:
      return StartFunni()
          .Until(std::bind(&StateSubsystem::IsControllerActive, this))
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
    default:
      return m_subsystems.led->Error()
          .AndThen(frc2::WaitCommand(1_s).ToPtr())
          .Until(std::bind(&StateSubsystem::IsControllerActive, this))
          .AndThen(SetState(RobotState::Manual))
          .AndThen(RunStateDeferred().ToPtr());
  }
}

frc2::CommandPtr StateSubsystem::StartIntaking() {
  return m_subsystems.led->ShowFromState([] { return RobotState::Intaking; })
      .AndThen(
          IntakingCommands::Intake(m_subsystems.intake)
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

  return m_subsystems.led
      ->ShowFromState([] { return RobotState::ScoringSpeaker; })
      .AndThen(PathFactory::GetPathFromFinalLocation(
          [] { return FinalLocation::Podium; }, m_subsystems.drive))
      // ? TODO: Snap to angle first?
      .AndThen(
          ScoringCommands::Score([] { return ScoringDirection::SpeakerSide; },
                                 m_subsystems.scoring, m_subsystems.intake))
      .WithTimeout(20_s);
}

frc2::CommandPtr StateSubsystem::StartScoringAmp() {
  /*
     Signal that we are scoring amp
     Go to the amp with PP on the fly path generation
     Orient the robot with the amp shooter facing the amp
     Shoot with timeout
  */
  return m_subsystems.led->ShowFromState([] { return RobotState::ScoringAmp; })
      .AndThen(PathFactory::GetPathFromFinalLocation(
          [] { return FinalLocation::Amp; }, m_subsystems.drive))
      // ? TODO: Snap to angle first?
      .AndThen(ScoringCommands::Score([] { return ScoringDirection::AmpSide; },
                                      m_subsystems.scoring,
                                      m_subsystems.intake))
      .WithTimeout(20_s);
}

frc2::CommandPtr StateSubsystem::StartScoringSubwoofer() {
  /*
     Signal that we are scoring subwoofer
     Go to the speaker with PP on the fly path generation
     Orient the robot with the amp shooter facing the speaker
     Shoot with timeout
  */
  return m_subsystems.led
      ->ShowFromState([] { return RobotState::ScoringSubwoofer; })
      .AndThen(PathFactory::GetPathFromFinalLocation(
          [] { return FinalLocation::Subwoofer; }, m_subsystems.drive))
      // ? TODO: Snap to angle first?
      .AndThen(
          ScoringCommands::Score([] { return ScoringDirection::Subwoofer; },
                                 m_subsystems.scoring, m_subsystems.intake))
      .WithTimeout(20_s);
}

frc2::CommandPtr StateSubsystem::StartManual() {
  /*
     Signal that we are in manual
     (Call after automated commands or after manual intervention from driver 1)
  */
  return m_subsystems.led->ShowFromState([this] {
    m_active = false;
    return RobotState::Manual;
  });
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
              [this] { return GetFinalFromState(); }, m_subsystems.drive
              // ExtendAbsolute(m_subsystems.leftClimb, m_subsystems.rightClimb)
              //  .ToPtr()))
              ))
      .AndThen(
          RetractClimbCommand(m_subsystems.leftClimb, m_subsystems.rightClimb)
              .ToPtr())
      .AndThen(BalanceCommand(m_subsystems.drive, m_subsystems.leftClimb,
                              m_subsystems.rightClimb)
                   .ToPtr())
      .WithTimeout(20_s);
}

frc2::CommandPtr StateSubsystem::StartSource() {
  /*
     Signal that we about to go to the source
     Use on the fly PP to go to approx. location
     Use pre-made PP path to orient robot and drive forward
  */

  return m_subsystems.led->Intaking()
      .AndThen(PathFactory::GetPathFromFinalLocation(
          [this] { return GetFinalFromState(); }, m_subsystems.drive))
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

  frc2::CommandPtr scoringCmd = frc2::InstantCommand([] {}).ToPtr();
  switch (m_currentState) {
    case RobotState::AutoSequenceAmp:
      scoringCmd = StartScoringAmp();
      break;
    case RobotState::AutoSequenceSpeaker:
      scoringCmd = StartScoringAmp();
      break;
    case RobotState::AutoSequenceSubwoofer:
      scoringCmd = StartScoringAmp();
      break;
    default:
      ConsoleLogger::getInstance().logError(
          "StateSubsystem", "Unsupported AutoSequence from state %d",
          static_cast<uint8_t>(m_currentState));
      m_subsystems.led->ErrorAsync();
  }

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

frc2::CommandPtr StateSubsystem::MoveToSourceAndIntake() {
  return StartSource().AndThen(
      FunniCommands::OuttakeUntilPresent(
          m_subsystems.intake, m_subsystems.scoring, ScoringDirection::AmpSide)
          // TODO: REMOVE THIS; ONLY FOR TESTING PURPOSES
          .WithTimeout(5_s));
}

bool StateSubsystem::IsControllerActive() {
  bool active = false;
  int count = m_driverController.GetButtonCount();
  for (int i = 1; i <= count; i++) {
    active |= m_driverController.GetRawButton(i);
  }
  count = m_driverController.GetAxisCount();
  for (int i = 0; i < count; i++) {
    active |=
        abs(m_driverController.GetRawAxis(i)) >= OIConstants::kDriveDeadband;
  }
  count = m_driverController.GetPOVCount();
  for (int i = 0; i < count; i++) {
    active |= m_driverController.GetPOV(i) != -1;
  }
  // Check the operator's "stop" button
  active |= m_operatorController.RightStick().Get();
  if (active) {
    ConsoleLogger::getInstance().logVerbose("StateSubsystem",
                                            "Controller interrupt! %s", "");
  }
  return active;
}