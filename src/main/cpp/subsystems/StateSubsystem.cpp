#include "subsystems/StateSubsystem.h"

#include "Constants.h"
#include "commands/Funni.h"
#include "commands/IntakeInCommand.h"
#include "commands/IntakeOutCommand.h"
#include "commands/ScoreAmpCommand.h"
#include "commands/ScoreSpeakerCommand.h"
#include "commands/ScoreSubwooferCommand.h"
#include "commands/ExtendClimbCommand.h"
#include "commands/BalanceCommand.h"
#include "commands/DriveVelocityCommand.h"
#include "commands/ExtendAbsoluteCommand.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "utils/ShuffleboardLogger.h"

StateSubsystem::StateSubsystem(Subsystems_t& subsystems) 
    : m_currentState(RobotState::Manual), m_subsystems{subsystems} {}

void StateSubsystem::IncrementState() {
    uint8_t nextState = ((uint8_t)m_currentState) + 1;
    m_currentState = (RobotState)(nextState % 6);
}

frc2::CommandPtr StateSubsystem::UpdateState(RobotState newState) {
    switch (newState) {
        case RobotState::Manual:
            return StartManual();
        case RobotState::ScoringSpeaker:
            return StartScoringSpeaker();
        case RobotState::ScoringAmp:
            return StartScoringAmp();
        case RobotState::ScoringSubwoofer:
            return StartScoringSubwoofer();
        case RobotState::Intaking:
            return StartIntaking();
        case RobotState::Climb:
            return StartClimb(1);
        default:
            return RunOnce([] {});
    }
}

frc2::CommandPtr StateSubsystem::StartIntaking() {
    return m_subsystems.led->ShowFromState([] { return RobotState::Intaking; })
    .AndThen(
        IntakeIn(m_subsystems.intake).ToPtr()
        .RaceWith(
            DriveVelocity(0.01_m, 0_deg, 2_mps, m_subsystems.drive).ToPtr().Repeatedly()
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

    return RunOnce([] {});
}