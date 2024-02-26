#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ScoringSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

class NoteShuffle : public frc2::CommandHelper<frc2::Command, NoteShuffle> {
 public:
  NoteShuffle(IntakeSubsystem* intake) : m_intake{intake}, isFinished{false} {
    AddRequirements(m_intake);
  }

  void Initialize() override {
    if (!m_intake->NotePresentUpper()) {
      isFinished = true;
      return;
    }
    isFinished = false;
  }

  void Execute() override {
    if (m_intake->NotePresentUpper()) {
      m_intake->Out(ScoringConstants::kShuffleSpeed);
       return;
    }
    m_intake->Stop();
    isFinished = true;
  }

  bool IsFinished() override { return isFinished; }

  void End() { m_intake->Stop(); }

 private:
  bool isFinished;
  IntakeSubsystem* m_intake;
};