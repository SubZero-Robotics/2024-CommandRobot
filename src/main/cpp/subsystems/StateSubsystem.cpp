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
#include "commands/ScoreAmpCommand.h"
#include "commands/ScoreSpeakerCommand.h"
#include "commands/ScoreSubwooferCommand.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "utils/ShuffleboardLogger.h"

using namespace AutoConstants::Locations;

StateSubsystem::StateSubsystem(Subsystems_t& subsystems,
                               frc2::CommandXboxController& controller)
    : m_currentState(RobotState::Manual),
      m_subsystems{subsystems},
      m_controller{controller} {}

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
    default:
      return m_subsystems.led->Error();
  }
}

frc2::CommandPtr StateSubsystem::StartIntaking() {
  return m_subsystems.led->ShowFromState([] { return RobotState::Intaking; })
      .AndThen(IntakeIn(m_subsystems.intake)
                   .ToPtr()
                   .RaceWith(DriveVelocity(0_deg, 2_mps, m_subsystems.drive)
                                 .ToPtr()
                                 .Repeatedly())
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
      .AndThen(ScoreSpeaker(m_subsystems.scoring, m_subsystems.intake).ToPtr())
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
      .AndThen(ScoreAmp(m_subsystems.scoring, m_subsystems.intake).ToPtr())
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
      .AndThen(ScoreAmp(m_subsystems.scoring, m_subsystems.intake).ToPtr())
      .WithTimeout(20_s);
}

frc2::CommandPtr StateSubsystem::StartManual() {
  /*
     Signal that we are in manual
     (Call after automated commands or after manual intervention from driver 1)
  */
  return m_subsystems.led->ShowFromState([] { return RobotState::Manual; });
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
              ExtendAbsolute(m_subsystems.leftClimb, m_subsystems.rightClimb)
                  .ToPtr()))
      .AndThen(
          RetractClimbCommand(m_subsystems.leftClimb, m_subsystems.rightClimb)
              .ToPtr())
      .AndThen(BalanceCommand(m_subsystems.drive, m_subsystems.leftClimb,
                              m_subsystems.rightClimb)
                   .ToPtr())
      .WithTimeout(20_s);
}

bool StateSubsystem::IsControllerActive() {
  bool active = false;
  int count = m_controller.GetButtonCount();
  for (int i = 1; i <= count; i++) {
    active |= m_controller.GetRawButton(i);
  }
  count = m_controller.GetAxisCount();
  for (int i = 0; i < count; i++) {
    active |= abs(m_controller.GetRawAxis(i)) >= OIConstants::kDriveDeadband;
  }
  count = m_controller.GetPOVCount();
  for (int i = 0; i < count; i++) {
    active |= m_controller.GetPOV(i) != -1;
  }
  return active;
}