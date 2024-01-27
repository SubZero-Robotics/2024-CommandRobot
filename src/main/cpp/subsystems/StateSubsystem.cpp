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
        case RobotState::Loaded:
            return StartLoading();
        case RobotState::Intaking:
            return StartIntaking();
        case RobotState::Climb:
            return StartClimb();
        default:
            return RunOnce([] {});
    }
}

frc2::CommandPtr StateSubsystem::StartIntaking() {
    
}

frc2::CommandPtr StateSubsystem::StartScoringSpeaker() {

}

frc2::CommandPtr StateSubsystem::StartScoringAmp() {

}

frc2::CommandPtr StateSubsystem::StartScoringSubwoofer() {

}

frc2::CommandPtr StateSubsystem::StartLoading() {

}

frc2::CommandPtr StateSubsystem::StartManual() {

}

frc2::CommandPtr StateSubsystem::StartClimb() {

}