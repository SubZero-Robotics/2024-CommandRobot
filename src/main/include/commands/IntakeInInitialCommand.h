#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/button/CommandXboxController.h>

#include "subsystems/IntakeSubsystem.h"

class IntakeInInitial
    : public frc2::CommandHelper<frc2::Command, IntakeInInitial> {
 public:
  /**
   * Creates a new Intake.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit IntakeInInitial(IntakeSubsystem* subsystem)
      : m_intake{subsystem}, isFinished{false} {
    // Register that this command requires the subsystem.
    AddRequirements(m_intake);
  }

  void Initialize() override {
    if (m_intake->NotePresent()) {
      isFinished = true;
      return;
    }

    isFinished = false;
  }

  void Execute() override {
    if (m_intake->NotePresentUpper() && !m_intake->NotePresentLower()) {
      m_intake->Stop();
      isFinished = true;
    }

    m_intake->In();
  }

  bool IsFinished() override { return isFinished; }

  void End(bool interrupted) { m_intake->Stop(); }

 private:
  IntakeSubsystem* m_intake;
  bool isFinished;
};