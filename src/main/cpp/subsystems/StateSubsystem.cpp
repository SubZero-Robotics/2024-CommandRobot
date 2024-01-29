#include "subsystems/StateSubsystem.h"

#include "Constants.h"
#include "commands/Funni.h"
#include "commands/IntakeInCommand.h"
#include "commands/IntakeOutCommand.h"
#include "commands/ScoreAmpCommand.h"
#include "commands/ScoreSpeakerCommand.h"
#include "commands/ScoreSubwooferCommand.h"
#include "commands/ExtendClimbCommand.h"
#include "commands/RetractClimbCommand.h"
#include "commands/BalanceCommand.h"
#include "commands/DriveVelocityCommand.h"
#include "commands/ExtendAbsoluteCommand.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "autos/PathFactory.h"
#include "utils/ShuffleboardLogger.h"

StateSubsystem::StateSubsystem(Subsystems_t& subsystems, frc2::CommandXboxController &controller)
    : m_currentState(RobotState::Manual), m_subsystems{subsystems}, m_controller{controller} {}

void StateSubsystem::IncrementState() {
    uint8_t nextState = ((uint8_t)m_currentState) + 1;
    m_currentState = (RobotState)(nextState % 6);
}

frc2::CommandPtr StateSubsystem::UpdateState(RobotState newState) {
    // TODO: Perform checks to ensure we can't enter the wrong state
    switch (newState) {
        case RobotState::Manual:
            return StartManual();
        case RobotState::ScoringSpeaker:
            return StartScoringSpeaker()
                .Until(std::bind(&StateSubsystem::IsControllerActive, this)).AndThen(StartManual());
        case RobotState::ScoringAmp:
            return StartScoringAmp()
                .Until(std::bind(&StateSubsystem::IsControllerActive, this)).AndThen(StartManual());
        case RobotState::ScoringSubwoofer:
            return StartScoringSubwoofer()
                .Until(std::bind(&StateSubsystem::IsControllerActive, this)).AndThen(StartManual());
        case RobotState::Intaking:
            return StartIntaking()
                .Until(std::bind(&StateSubsystem::IsControllerActive, this)).AndThen(StartManual());
        case RobotState::Climb:
            return StartClimb(1)
                .Until(std::bind(&StateSubsystem::IsControllerActive, this)).AndThen(StartManual());
        default:
            return m_subsystems.led->Error();
    }
}

frc2::CommandPtr StateSubsystem::StartIntaking() {
    return m_subsystems.led->ShowFromState([] { return RobotState::Intaking; })
    .AndThen(
        IntakeIn(m_subsystems.intake).ToPtr()
        .RaceWith(
            DriveVelocity(0_deg, 2_mps, m_subsystems.drive).ToPtr().Repeatedly()
        )
        .WithTimeout(5_s)
    );
}

frc2::CommandPtr StateSubsystem::StartScoringSpeaker() {
    /* 
       Signal that we are scoring speaker
       Go to the speaker with PP on the fly path generation
       Orient the robot with the speaker shooter facing the speaker
       Shoot with timeout
    */

    return m_subsystems.led->ShowFromState([] { return RobotState::ScoringSpeaker; })
    .AndThen(
        PathFactory::GetPathFromFinalLocation([] { return FinalLocation::Podium; }, m_subsystems.drive)
    )
    .AndThen(
        ScoreSpeaker(m_subsystems.scoring, m_subsystems.intake).ToPtr()
    );
}

frc2::CommandPtr StateSubsystem::StartScoringAmp() {
    /* 
       Signal that we are scoring amp
       Go to the amp with PP on the fly path generation
       Orient the robot with the amp shooter facing the amp
       Shoot with timeout
    */
    return m_subsystems.led->ShowFromState([] { return RobotState::ScoringAmp; })
    .AndThen(
        PathFactory::GetPathFromFinalLocation([] { return FinalLocation::Amp; }, m_subsystems.drive)
    )
    .AndThen(
        ScoreAmp(m_subsystems.scoring, m_subsystems.intake).ToPtr()
    );
}

frc2::CommandPtr StateSubsystem::StartScoringSubwoofer() {
    /* 
       Signal that we are scoring subwoofer
       Go to the speaker with PP on the fly path generation
       Orient the robot with the amp shooter facing the speaker
       Shoot with timeout
    */
    return m_subsystems.led->ShowFromState([] { return RobotState::ScoringSubwoofer; })
    .AndThen(
        PathFactory::GetPathFromFinalLocation([] { return FinalLocation::Subwoofer; }, m_subsystems.drive)
    )
    .AndThen(
        ScoreAmp(m_subsystems.scoring, m_subsystems.intake).ToPtr()
    );
}

frc2::CommandPtr StateSubsystem::StartManual() {
    /*
       Signal that we are in manual
       (Call after automated commands or after manual intervention from driver 1)
    */
    return m_subsystems.led->ShowFromState([] { return RobotState::Manual; });
}

frc2::CommandPtr StateSubsystem::StartClimb(uint8_t stageLocation) {
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
        PathFactory::GetPathFromFinalLocation([] { return FinalLocation::StageLeft; },
            m_subsystems.drive, ExtendAbsolute(m_subsystems.leftClimb, m_subsystems.rightClimb).ToPtr())
    )
    .AndThen(
        RetractClimbCommand(m_subsystems.leftClimb, m_subsystems.rightClimb).ToPtr()
    )
    .AndThen(
        BalanceCommand(m_subsystems.drive, m_subsystems.leftClimb, m_subsystems.rightClimb).ToPtr()
    );
}

// Is there an easier way to do this?
bool StateSubsystem::IsControllerActive() {
    return  m_controller.GetAButtonPressed() ||
            m_controller.GetBButtonPressed() ||
            m_controller.GetYButtonPressed() ||
            m_controller.GetXButtonPressed() ||
            m_controller.GetLeftBumperPressed() ||
            m_controller.GetRightBumperPressed() ||
            abs(m_controller.GetLeftTriggerAxis()) >= OIConstants::kDriveDeadband ||
            abs(m_controller.GetRightTriggerAxis()) >= OIConstants::kDriveDeadband ||
            abs(m_controller.GetLeftX()) >= OIConstants::kDriveDeadband ||
            abs(m_controller.GetLeftY()) >= OIConstants::kDriveDeadband ||
            abs(m_controller.GetRightX()) >= OIConstants::kDriveDeadband ||
            abs(m_controller.GetRightY()) >= OIConstants::kDriveDeadband;
}