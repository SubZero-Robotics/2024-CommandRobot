#pragma once

#include <frc2/command/DeferredCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandScheduler.h>

class StateManager : public frc2::SubsystemBase {
 public:
  StateManager();

  /**
   * Register a new state
   *
   * @param StateName The name of the state
   * @param Entrypoint A CommandPtr that will run while the state is active
   * @param Timeout A timeout for the Entrypoint, after it has elapsed the state
   * will be canceled
   */
  void RegisterState(std::string StateName, frc2::CommandPtr Entrypoint,
                     units::second_t Timeout);

  /**
   * Register a new state
   *
   * @param StateName The name of the state
   * @param Entrypoint A CommandPtr that will run while the state is active
   */
  void RegisterState(std::string StateName, frc2::CommandPtr Entrypoint);

  /**
   * Register a state that should be ran when no other state is active, for instance a manual mode
   * 
   * @param StateName The name of the default state
   * @param Entrypoint A CommandPtr to the default state
  */
  void SetDefaultState(std::string StateName, frc2::CommandPtr Entrypoint);

  /**
   * Set a global interrupt to cancel states. Will be called every robot loop to
   * check if states should be canceled
   *
   * @param Interrupt A method to check if the state should be canceled, for
   * instance canceling the current state if a button is pressed
   */
  void SetInterrupt(const std::function<int()> Interrupt);

  /**
   * Returns a CommandPtr for the state with management attached, may be bound
   * to a button to schedule the state.
   *
   * @param StateName The name of the state to run
   *
   * @returns The CommandPtr to invoke the state
   */
  frc2::CommandPtr GetStateCommand(std::string StateName);

  frc2::DeferredCommand RunStateDeferred();

  void RunState();

  void SetDesiredState();

  /**
   * @returns The name of the currently active state
  */
  std::string GetState();

 private:
  std::vector<std::string, frc2::CommandPtr> m_cmds{};
};