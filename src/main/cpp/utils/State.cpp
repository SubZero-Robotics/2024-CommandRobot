#include "utils/State.h"

StateManager::StateManager() : m_currentState(RobotState::Idling) {}

void StateManager::incrementState() {
  uint8_t nextState = ((uint8_t)m_currentState) + 1;
  m_currentState = (RobotState)(nextState % 5);
}

bool StateManager::updateState(RobotState newState) {
  m_currentState = newState;

  // TODO: check if state change is valid
  return true;
}