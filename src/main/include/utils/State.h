#pragma once

#include <functional>

#include "Constants.h"

enum class RobotState {
    Intaking = 0,
    ScoringSpeaker,
    ScoringAmp,
    Stowing,
    Idling
};

typedef std::function<RobotState ()> StateGetter;

class StateManager {
    public:
        StateManager();

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

        inline RobotState getState() const {
            return m_currentState;
        };

    private:
        RobotState m_currentState;

        bool updateState(RobotState newState);
};