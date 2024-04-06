#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ScoringSubsystem.h"

class NoteShuffle : public frc2::CommandHelper<frc2::Command, NoteShuffle> {
 public:
  explicit NoteShuffle(IntakeSubsystem* intake)
      : isFinished{false}, m_intake{intake} {
    AddRequirements(m_intake);
  }

  void Initialize() override { isFinished = false; }

  void Execute() override { m_intake->Out(ScoringConstants::kShuffleSpeed); }

  bool IsFinished() override { return m_intake->NotePresentLower(); }

  void End() {}

 private:
  bool isFinished;
  IntakeSubsystem* m_intake;
};