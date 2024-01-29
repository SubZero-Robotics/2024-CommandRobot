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
#include "commands/ScoreAmpCommand.h"
#include "commands/ScoreSpeakerCommand.h"
#include "commands/ScoreSubwooferCommand.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LedSubsystem.h"
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
  StateSubsystem(Subsystems_t &subsystems, frc2::CommandXboxController &);

  void IncrementState();

  frc2::CommandPtr UpdateState(RobotState newState);

  frc2::CommandPtr StartIntaking();

  frc2::CommandPtr StartScoringSpeaker();

  frc2::CommandPtr StartScoringAmp();

  frc2::CommandPtr StartScoringSubwoofer();

  frc2::CommandPtr StartManual();

  frc2::CommandPtr StartClimb(uint8_t stageLocation);

  inline RobotState GetState() const { return m_currentState; }

 private:
  bool IsControllerActive();

  RobotState m_currentState;
  Subsystems_t &m_subsystems;
  frc2::CommandXboxController &m_controller;
};