#ifndef STATE_H
#define STATE_H

#include "Constants.h"

enum class RobotState {
    Intaking,
    ScoringSpeaker,
    ScoringAmp,
    Stowing,
    Idling
};

class StateManager {
    public:
        StateManager() {
            currentState = RobotState::Idling;
        };

        bool startIntaking() {
            return updateState(RobotState::Intaking);
        };

        bool startScoringSpeaker() {
            return updateState(RobotState::ScoringSpeaker);
        };

        bool startScoringAmp() {
            return updateState(RobotState::ScoringAmp);
        };

        bool startStowing() {
            return updateState(RobotState::Stowing);
        };

        bool startIdling() {
            return updateState(RobotState::Idling);
        };

        RobotState getState() {
            return currentState;
        };

    private:
        RobotState currentState;

        bool updateState(RobotState newState) {
            currentState = newState;

            // TODO: check if state change is valid
            return true;
        };
};

#endif