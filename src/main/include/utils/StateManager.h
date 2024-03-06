#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/DeferredCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>

template <typename T>
class StateManager : public frc2::SubsystemBase {
 public:
  StateManager(const std::function<bool()> Interrupt, T defaultState,
               frc2::CommandPtr&& defaultCommand)
      : m_currentState(defaultState),
        m_defaultState(defaultState),
        m_defaultCommand(std::move(defaultCommand)){};

  /**
   * Register a new state
   *
   * @param T A type that represents the state
   * @param command A CommandPtr that will run while the state is active
   * @param Timeout A timeout for the command, after it has elapsed the state
   * will be canceled
   */
  void RegisterState(T State, frc2::CommandPtr&& command,
                     units::second_t timeout, bool returnToDefault = false) {
    auto cmd = std::move(command).WithTimeout(timeout);
    if (returnToDefault) {
      cmd = std::move(cmd).AndThen(SetState(m_defaultState));
    }
    m_cmds[State] = std::move(cmd);
  };

  /**
   * Register a new state
   *
   * @param T A type that represents the state
   * @param command A CommandPtr that will run while the state is active
   */
  void RegisterState(T State, frc2::CommandPtr&& command,
                     bool returnToDefault = true) {
    auto cmd = std::move(command);
    if (returnToDefault) {
      cmd = cmd.AndThen(SetState(m_defaultState));
    }
    m_cmds[State] = cmd;
  };

  /**
   * Register a new state
   *
   * @param T A type that represents the state
   * @param action A std::function that will be converted into a state
   */
  void RegisterState(T State, std::function<void()> action,
                     bool returnToDefault = true) {
    auto cmd = frc2::InstantCommand(action).ToPtr();
    if (returnToDefault) {
      cmd = cmd.AndThen(SetState(m_defaultState));
    }
    m_cmds[State] = cmd;
  };

  /**
   * Returns a CommandPtr for the state with management attached, may be bound
   * to a button to schedule the state.
   *
   * @param T The state to get the command for
   *
   * @returns The CommandPtr to invoke the state
   */
  frc2::CommandPtr GetStateCommand(T state) { return m_cmds[state]; };

  /**
   * Returns a CommandPtr for the current state with management attached
   *
   * @returns The CommandPtr to schedule the state
   */
  frc2::CommandPtr GetStateCommand() {
    return std::move(m_cmds[m_currentState]);
  };

  /**
   * Set the state to a new state
   *
   * @param newState The new state to set
   *
   * @returns A DeferredCommand that will set the state and run the state
   */
  frc2::CommandPtr SetState(T newState) {
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
  frc2::DeferredCommand GetStateDeferred() {
    return frc2::DeferredCommand([this] { GetStateCommand(); }, {this});
  };

  /**
   * Schedule the current state
   */
  void ScheduleCurrentState() {
    frc2::CommandScheduler::GetInstance().Cancel(m_currentCommand);
    m_currentCommand = GetStateCommand();
    frc2::CommandScheduler::GetInstance().Schedule(m_currentCommand);
    m_active = true;
  };

  /**
   * @returns The indentifier for the current state
   */
  T GetState() { return m_currentState; };

  bool IsStateActive() { return m_active; };

  /**
   * Cancel the current state and set active to false
   */
  void CancelState() {
    frc2::CommandScheduler::GetInstance().Cancel(m_currentCommand);
    m_currentCommand = frc2::InstantCommand([] {}).ToPtr();
    m_active = false;
  };

 private:
  std::unordered_map<T, frc2::CommandPtr> m_cmds;

  T m_currentState;
  frc2::CommandPtr m_currentCommand = frc2::InstantCommand([] {}).ToPtr();
  bool m_active = false;
  T m_defaultState;
  frc2::CommandPtr m_defaultCommand;
};