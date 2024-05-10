#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <functional>

#include "Constants.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ScoringSubsystem.h"

class FlywheelRamp : public frc2::CommandHelper<frc2::Command, FlywheelRamp> {
 public:
  explicit FlywheelRamp(IntakeSubsystem* intake, ScoringSubsystem* scoring,
                        std::function<ScoringDirection()> direction)
      : isFinished{false},
        m_intake{intake},
        m_scoring{scoring},
        m_direction{direction} {
          AddRequirements({m_intake, m_scoring});
        }

  void Initialize() override {
    ConsoleWriter.logVerbose("Score State", "Start state: %s",
                                            "Ramping");
    m_scoring->StartScoringRamp(m_direction());
    isFinished = false;
  }

  void Execute() override {
    isFinished = true;
  }

  bool IsFinished() override { return isFinished; }

  void End(bool interrupted) override {
    ConsoleWriter.logVerbose("Score State", "End state: %s",
                                            "Ramping");
  }

 private:
  bool isFinished = false;
  IntakeSubsystem* m_intake;
  ScoringSubsystem* m_scoring;
  std::function<ScoringDirection()> m_direction;
};