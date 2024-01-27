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

void StateSubsystem::incrementState() {
    uint8_t nextState = ((uint8_t)m_currentState) + 1;
    m_currentState = (RobotState)(nextState % 6);
}

frc2::CommandPtr StateSubsystem::updateState(RobotState newState) {
    switch (newState) {
        case RobotState::Manual:
            return startManual();
        case RobotState::ScoringSpeaker:
            return startScoringSpeaker();
        case RobotState::ScoringAmp:
            return startScoringAmp();
        case RobotState::ScoringSubwoofer:
            return startScoringSubwoofer();
        case RobotState::Loaded:
            return startLoading();
        case RobotState::Intaking:
            return startIntaking();
        case RobotState::Climb:
            return startClimb();
        default:
            return RunOnce([] {});
    }
}

frc2::CommandPtr StateSubsystem::startIntaking() {
    
}

frc2::CommandPtr StateSubsystem::startScoringSpeaker() {

}

frc2::CommandPtr StateSubsystem::startScoringAmp() {

}

frc2::CommandPtr StateSubsystem::startScoringSubwoofer() {

}

frc2::CommandPtr StateSubsystem::startLoading() {

}

frc2::CommandPtr StateSubsystem::startManual() {

}

frc2::CommandPtr StateSubsystem::startClimb() {

}