#pragma once

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subzero/moduledrivers/ConnectorX.h>

class GamepieceFunni
    : public frc2::CommandHelper<frc2::Command, GamepieceFunni> {
 public:
  /**
   * Creates a new Intake.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit GamepieceFunni(ConnectorX::ConnectorXBoard* leds)
      : m_leds{leds}, isFinished{false} {
    // Register that this command requires the subsystem.
  }

  void Initialize() override {
    m_leds->setOn();
    m_leds->setPattern(ConnectorX::LedPort::P0,
                       ConnectorX::PatternType::RGBFade, false, 22);
  }

  void Execute() override {}

  bool IsFinished() override { return isFinished; }

  void End(bool interrupted) override { m_leds->setOff(); }

 private:
  ConnectorX::ConnectorXBoard* m_leds;
  uint8_t state = 0;
  bool isFinished = false;
};