#include "utils/StateManager.h"

template <typename StateType>
StateManager<StateType>::StateManager(const std::function<bool()> Interrupt,
                                      StateType DefaultState,
                                      frc2::CommandPtr DefaultEntrypoint)
    : m_currentState(DefaultState) {}

template <typename StateType>
void StateManager<StateType>::RegisterState(StateType State,
                                            frc2::CommandPtr Entrypoint,
                                            units::second_t Timeout) {
  m_cmds[State] = std::move(Entrypoint).WithTimeout(Timeout);
}

template <typename StateType>
void StateManager<StateType>::RegisterState(StateType State,
                                            frc2::CommandPtr Entrypoint) {
  m_cmds[State] = std::move(Entrypoint);
}

template <typename StateType>
frc2::CommandPtr StateManager<StateType>::GetStateCommand(StateType State) {
  return m_cmds[State];
}

template <typename StateType>
frc2::CommandPtr StateManager<StateType>::SetState(StateType newState) {
  return frc2::InstantCommand([this, newState] {
           m_currentState = newState;
           RunState();
         })
      .ToPtr();
}

template <typename StateType>
frc2::DeferredCommand StateManager<StateType>::RunStateDeferred() {
  return frc2::DeferredCommand([this] { RunState(); }, {this});
}

template <typename StateType>
frc2::CommandPtr StateManager<StateType>::RunState() {
  return m_cmds[m_currentState];
}

template <typename StateType>
void StateManager<StateType>::SetDesiredState() {
  frc2::CommandScheduler::GetInstance().Cancel(m_cmd);
  m_cmd = RunState();
  frc2::CommandScheduler::GetInstance().Schedule(m_cmd);
  m_active = true;
}