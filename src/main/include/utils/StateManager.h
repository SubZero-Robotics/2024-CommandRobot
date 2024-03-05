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
               frc2::CommandPtr DefaultEntrypoint);

  /**
   * Register a new state
   *
   * @param StateType A type that represents the state
   * @param Entrypoint A CommandPtr that will run while the state is active
   * @param Timeout A timeout for the Entrypoint, after it has elapsed the state
   * will be canceled
   */
  void RegisterState(StateType State, frc2::CommandPtr Entrypoint,
                     units::second_t Timeout);

  /**
   * Register a new state
   *
   * @param StateType A type that represents the state
   * @param Entrypoint A CommandPtr that will run while the state is active
   */
  void RegisterState(StateType State, frc2::CommandPtr Entrypoint);

  /**
   * Returns a CommandPtr for the state with management attached, may be bound
   * to a button to schedule the state.
   *
   * @param StateType The state to get the command for
   *
   * @returns The CommandPtr to invoke the state
   */
  frc2::CommandPtr GetStateCommand(StateType State);

  /**
   * Set the state to a new state
   *
   * @param newState The new state to set
   *
   * @returns A DeferredCommand that will set the state and run the state
   */
  frc2::CommandPtr SetState(StateType newState);

  /**
   * Run the current state
   *
   * @returns A DeferredCommand that will run the state
   */
  frc2::DeferredCommand RunStateDeferred();

  /**
   * Run the current state
   */
  frc2::CommandPtr RunState();

  /**
   * Set the desired state
   */
  void SetDesiredState();

  /**
   * @returns The indentifier for the current state
   */
  StateType GetState();

 private:
  std::unordered_map<StateType, frc2::CommandPtr> m_cmds{};

  StateType m_currentState;
  frc2::CommandPtr m_cmd = frc2::InstantCommand([] {}).ToPtr();
  bool m_active = false;
};