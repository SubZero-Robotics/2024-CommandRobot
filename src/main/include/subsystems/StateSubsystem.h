#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <rev/CANSparkFlex.h>

#include <functional>

#include "ColorConstants.h"
#include "Constants.h"
#include "autos/PathFactory.h"
#include "commands/BalanceCommand.h"
#include "commands/ExtendClimbCommand.h"
#include "commands/Funni.h"
#include "commands/IntakeInCommand.h"
#include "commands/IntakeOutCommand.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LedSubsystem.h"
#include "subsystems/ScoringSubsystem.h"
#include "utils/ShuffleboardLogger.h"

typedef struct {
  DriveSubsystem *drive;
  ClimbSubsystem *leftClimb;
  ClimbSubsystem *rightClimb;
  IntakeSubsystem *intake;
  ScoringSubsystem *scoring;
  LedSubsystem *led;
} Subsystems_t;

class StateSubsystem : public frc2::SubsystemBase {
 public:
  StateSubsystem(Subsystems_t &subsystems, frc2::CommandXboxController &driver,
                 frc2::CommandXboxController &op);

  frc2::CommandPtr SetState(RobotState newState) {
    return frc2::InstantCommand(
               [this, newState] {
                 ConsoleLogger::getInstance().logVerbose(
                     "StateSubsystem", "Setting new state to %u",
                     (uint8_t)newState);
                 m_currentState = newState;
               },
               {this})
        .ToPtr();
  }

  void IncrementState();

  frc2::DeferredCommand RunStateDeferred() {
    return frc2::DeferredCommand([this] { return RunState(); }, {this});
  }

  /**
   * Run the FSM using the current state
   */
  frc2::CommandPtr RunState();

  frc2::CommandPtr StartIntaking();

  frc2::CommandPtr StartScoringSpeaker();

  frc2::CommandPtr StartScoringAmp();

  frc2::CommandPtr StartScoringSubwoofer();

  frc2::CommandPtr StartManual();

  frc2::CommandPtr StartClimb();

  inline RobotState GetState() const { return m_currentState; }

 private:
  bool IsControllerActive();

  inline FinalLocation GetFinalFromState() {
    switch (m_currentState) {
      case RobotState::ClimbStageLeft:
        return FinalLocation::StageLeft;
      case RobotState::ClimbStageCenter:
        return FinalLocation::CenterStage;
      case RobotState::ClimbStageRight:
        return FinalLocation::StageRight;
      default:
        return FinalLocation::Source1;
    }
  }

  RobotState m_currentState;
  Subsystems_t &m_subsystems;
  frc2::CommandXboxController &m_driverController;
  frc2::CommandXboxController &m_operatorController;
};