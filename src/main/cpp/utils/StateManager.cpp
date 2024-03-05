#include "utils/StateManager.h"

template <typename StateType>
StateManager<StateType>::StateManager(const std::function<bool()> Interrupt,
                                      StateType DefaultState,
                                      frc2::CommandPtr DefaultEntrypoint)
    : m_currentState(DefaultState),
      m_defaultState(DefaultState),
      m_defaultEntryPoint(DefaultEntrypoint) {}

template <typename StateType>
void StateManager<StateType>::RegisterState(StateType State,
                                            frc2::CommandPtr Entrypoint,
                                            units::second_t Timeout,
                                            bool ReturnToDefault) {
  auto cmd = std::move(Entrypoint).WithTimeout(Timeout);
  if (ReturnToDefault) {
    cmd = cmd.AndThen(SetState(m_defaultState));
  }
  m_cmds[State] = std::move(cmd);
}

template <typename StateType>
void StateManager<StateType>::RegisterState(StateType State,
                                            frc2::CommandPtr Entrypoint,
                                            bool ReturnToDefault) {
  auto cmd = std::move(Entrypoint);
  if (ReturnToDefault) {
    cmd = cmd.AndThen(SetState(m_defaultState));
  }
  m_cmds[State] = std::move(cmd);
}

template <typename StateType>
void StateManager<StateType>::RegisterState(StateType State,
                                            std::function<void()> Entrypoint,
                                            bool ReturnToDefault) {
  auto cmd = frc2::InstantCommand(Entrypoint).ToPtr();
  if (ReturnToDefault) {
    cmd = cmd.AndThen(SetState(m_defaultState));
  }
  m_cmds[State] = cmd;
}

template <typename StateType>
frc2::CommandPtr StateManager<StateType>::GetStateCommand(StateType State) {
  return m_cmds[State];
}

template <typename StateType>
frc2::CommandPtr StateManager<StateType>::GetStateCommand() {
  return m_cmds[m_currentState];
}

template <typename StateType>
frc2::CommandPtr StateManager<StateType>::SetState(StateType newState) {
  return frc2::InstantCommand([this, newState] {
           m_currentState = newState;
           GetStateCommand();
         })
      .ToPtr();
}

template <typename StateType>
frc2::DeferredCommand StateManager<StateType>::RunStateDeferred() {
  return frc2::DeferredCommand([this] { GetStateCommand(); }, {this});
}

template <typename StateType>
void StateManager<StateType>::ScheduleCurrentState() {
  frc2::CommandScheduler::GetInstance().Cancel(m_cmd);
  m_cmd = GetStateCommand();
  frc2::CommandScheduler::GetInstance().Schedule(m_cmd);
  m_active = true;
}

template <typename StateType>
bool StateManager<StateType>::IsStateActive() {
  return m_active;
}