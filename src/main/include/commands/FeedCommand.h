#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "Constants.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ScoringSubsystem.h"

class Feed : public frc2::CommandHelper<frc2::Command, Feed> {
 public:
  explicit Feed(IntakeSubsystem* intake, ScoringSubsystem* scoring,
                   ScoringDirection direction)
      : isFinished{false},
        m_intake{intake},
        m_scoring{scoring},
        m_direction{direction} {}

  void Initialize() override {
    ConsoleLogger::getInstance().logVerbose("Score State", "Start state: %s",
                                            "Feed");
    m_scoring->SpinVectorSide(m_direction);
    m_intake->In();
    isFinished = false;
  }

  void Execute() override {
    isFinished = !m_scoring->GetMotorFreeWheel(m_direction);
  }

  bool IsFinished() override { return isFinished; }

  void End(bool interrupted) override {
    ConsoleLogger::getInstance().logVerbose("Score State", "End state: %s",
                                            "Feed");
  }

 private:
  bool isFinished = false;
  IntakeSubsystem* m_intake;
  ScoringSubsystem* m_scoring;
  ScoringDirection m_direction;
};