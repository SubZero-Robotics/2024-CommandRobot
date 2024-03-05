#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/DeferredCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>

template <typename StateType>
class StateManager : public frc2::SubsystemBase {
 public:
  StateManager(const std::function<bool()> Interrupt, StateType DefaultState,
               frc2::CommandPtr&& DefaultEntrypoint)
      : m_currentState(DefaultState),
        m_defaultState(DefaultState),
        m_defaultEntryPoint(DefaultEntrypoint){};

  /**
   * Register a new state
   *
   * @param StateType A type that represents the state
   * @param Entrypoint A CommandPtr that will run while the state is active
   * @param Timeout A timeout for the Entrypoint, after it has elapsed the state
   * will be canceled
   */
  void RegisterState(StateType State, frc2::CommandPtr&& Entrypoint,
                     units::second_t Timeout, bool ReturnToDefault = false) {
    auto cmd = std::move(Entrypoint).WithTimeout(Timeout);
    if (ReturnToDefault) {
      cmd = cmd.AndThen(SetState(m_defaultState));
    }
    m_cmds[State] = cmd;
  };

  /**
   * Register a new state
   *
   * @param StateType A type that represents the state
   * @param Entrypoint A CommandPtr that will run while the state is active
   */
  void RegisterState(StateType State, frc2::CommandPtr&& Entrypoint,
                     bool ReturnToDefault = false) {
    auto cmd = std::move(Entrypoint);
    if (ReturnToDefault) {
      cmd = cmd.AndThen(SetState(m_defaultState));
    }
    m_cmds[State] = cmd;
  };

  /**
   * Register a new state
   *
   * @param StateType A type that represents the state
   * @param Entrypoint A std::function that will run while the state is active
   */
  void RegisterState(StateType State, std::function<void()> Entrypoint,
                     bool ReturnToDefault = false) {
    auto cmd = frc2::InstantCommand(Entrypoint).ToPtr();
    if (ReturnToDefault) {
      cmd = cmd.AndThen(SetState(m_defaultState));
    }
    m_cmds[State] = cmd;
  };

  /**
   * Returns a CommandPtr for the state with management attached, may be bound
   * to a button to schedule the state.
   *
   * @param StateType The state to get the command for
   *
   * @returns The CommandPtr to invoke the state
   */
  frc2::CommandPtr GetStateCommand(StateType State) { return m_cmds[State]; };

  /**
   * Returns a CommandPtr for the current state with management attached
   *
   * @returns The CommandPtr to schedule the state
   */
  frc2::CommandPtr GetStateCommand() { return m_cmds[m_currentState]; };

  /**
   * Set the state to a new state
   *
   * @param newState The new state to set
   *
   * @returns A DeferredCommand that will set the state and run the state
   */
  frc2::CommandPtr SetState(StateType newState) {
    return frc2::InstantCommand([this, newState] {
             m_currentState = newState;
             GetStateCommand();
           })
        .ToPtr();
  };

  /**
   * Run the current state
   *
   * @returns A DeferredCommand that will run the state
   */
  frc2::DeferredCommand RunStateDeferred() {
    return frc2::DeferredCommand([this] { GetStateCommand(); }, {this});
  };

  /**
   * Schedule the current state
   */
  void ScheduleCurrentState() {
    frc2::CommandScheduler::GetInstance().Cancel(m_cmd);
    m_cmd = GetStateCommand();
    frc2::CommandScheduler::GetInstance().Schedule(m_cmd);
    m_active = true;
  };

  /**
   * @returns The indentifier for the current state
   */
  StateType GetState() { return m_currentState; };

  bool IsStateActive() { return m_active; };

 private:
  std::unordered_map<StateType, frc2::CommandPtr> m_cmds;

  StateType m_currentState;
  frc2::CommandPtr m_cmd = frc2::InstantCommand([] {}).ToPtr();
  bool m_active = false;
  StateType m_defaultState;
  frc2::CommandPtr&& m_defaultEntryPoint;
};