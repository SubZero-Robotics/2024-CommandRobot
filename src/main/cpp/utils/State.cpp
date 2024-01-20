#include "utils/State.h"

StateManager::StateManager() : m_currentState(RobotState::Idling) {}

bool StateManager::updateState(RobotState newState) {
    m_currentState = newState;

    // TODO: check if state change is valid
    return true;
}