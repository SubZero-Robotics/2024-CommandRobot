#pragma once

#include <frc/BuiltInAccelerometer.h>
#include <frc/Notifier.h>
#include <frc/util/Color8Bit.h>
#include <frc2/command/DeferredCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/WaitCommand.h>

#include <chrono>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Constants.h"
#include "moduledrivers/ConnectorX.h"

class LedSubsystem : public frc2::SubsystemBase {
 public:
  LedSubsystem() : m_connectorX(ConnectorX::ConnectorXBoard(kLedAddress)) {
    ConsoleWriter.logVerbose("LedSubsystem", "LEDs init%s", "");
    createZones(ConnectorX::LedPort::P0, std::move(m_ledZones0));
    createZones(ConnectorX::LedPort::P1, std::move(m_ledZones1));
    m_connectorX.setOn();
  }

  void Periodic() override;

  void SimulationPeriodic() override;

  frc2::DeferredCommand GetDeferredFromState(StateGetter);
  frc2::CommandPtr ShowFromState(StateGetter);

  frc2::CommandPtr Intaking();
  // I really think it's outtaking with two T's...
  frc2::CommandPtr Outaking();
  frc2::CommandPtr ScoringSpeaker();
  frc2::CommandPtr ScoringAmp();
  frc2::CommandPtr ScoringSubwoofer();
  frc2::CommandPtr Loaded();
  frc2::CommandPtr Idling();
  frc2::CommandPtr Climbing();
  frc2::CommandPtr Funni();
  frc2::CommandPtr Error();
  frc2::CommandPtr AngryFace();
  frc2::CommandPtr HappyFace();
  frc2::CommandPtr BlinkingFace();
  frc2::CommandPtr SurprisedFace();
  frc2::CommandPtr AmogusFace();
  frc2::CommandPtr AimbotEnabled();
  frc2::CommandPtr OnTheFlyPP();
  frc2::CommandPtr VisionNoteDetected();
  frc2::CommandPtr SuccessfulIntake();

  void IdlingAsync();
  void ErrorAsync();

 private:
  enum class LedZone {
    LeftClimber = 0,
    Back,
    RightClimber,
    Front,
  };

  enum class EyePattern {
    Angry = 8,
    Happy = 9,
    Blinking = 10,
    Surprised = 11,
    Amogus = 12,
  };

  frc2::CommandPtr setZoneColorPattern(LedZone zone, ConnectorX::LedPort port,
                                       frc::Color8Bit color,
                                       ConnectorX::PatternType pattern,
                                       bool oneShot = false, int16_t delay = -1,
                                       bool reversed = false);

  void setZoneColorPatternAsync(LedZone zone, ConnectorX::LedPort port,
                                frc::Color8Bit color,
                                ConnectorX::PatternType pattern,
                                bool oneShot = false, int16_t delay = -1,
                                bool reversed = false);

  void delaySeconds(units::second_t delaySeconds);

  void showFace(EyePattern pattern);

  void createZones(ConnectorX::LedPort port,
                   std::vector<ConnectorX::Commands::NewZone> &&zones);

  void syncAllZones();

  ConnectorX::ConnectorXBoard m_connectorX;

  frc::BuiltInAccelerometer m_accel;

  // TODO: make into a constant elsewhere
  const uint16_t m_totalLeds = 67;
  std::vector<ConnectorX::Commands::NewZone> m_ledZones0 = {
      {.offset = 0, .count = 18},
      {.offset = 18, .count = 32},
      {.offset = 50, .count = 17},
  };
  std::vector<ConnectorX::Commands::NewZone> m_ledZones1 = {
      {.offset = 0, .count = 1},
      {.offset = 1, .count = 256},
  };
  std::vector<ConnectorX::Commands::NewZone> m_ledZone2 = {
      {.offset = 50, .count = 32},
  };
};