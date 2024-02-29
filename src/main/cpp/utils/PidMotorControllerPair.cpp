#include "utils/PidMotorControllerPair.h"

PidMotorControllerPair::PidMotorControllerPair(
    std::string name, rev::SparkPIDController& controllerFirst,
    rev::SparkPIDController& controllerSecond, PidSettings pidSettings,
    units::revolutions_per_minute_t maxRpm)
    : m_shuffleboardName{name},
      m_controllerFirst{controllerFirst},
      m_controllerSecond{controllerSecond},
      m_maxRpm{maxRpm} {
  // Doing it here so the PID controllers themselves get updated
  UpdatePidSettings(pidSettings);
}

void PidMotorControllerPair::RunWithVelocity(
    units::revolutions_per_minute_t rpmFirst,
    units::revolutions_per_minute_t rpmSecond) {
  m_controllerFirst.SetReference(rpmFirst.value(),
                                 rev::CANSparkBase::ControlType::kVelocity);
  m_controllerSecond.SetReference(rpmSecond.value(),
                                  rev::CANSparkBase::ControlType::kVelocity);
}

void PidMotorControllerPair::RunWithVelocity(double percentageFirst,
                                             double percentageSecond) {
  if (abs(percentageFirst) > 1.0 ||
      abs(percentageSecond) > 1.0) {
    ConsoleLogger::getInstance().logError(
        "PidMotorControllerPair",
        "Incorrect percentages for motor pair %s: First=%.4f | Second=%.4f",
        m_shuffleboardName.c_str(), percentageFirst, percentageSecond);
    return;
  }
  auto rpmFirst = units::revolutions_per_minute_t(m_maxRpm) * percentageFirst;
  auto rpmSecond = units::revolutions_per_minute_t(m_maxRpm) * percentageSecond;

  RunWithVelocity(rpmFirst, rpmSecond);
}

void PidMotorControllerPair::Stop() { RunWithVelocity(0, 0); }

void PidMotorControllerPair::UpdatePidSettings(PidSettings settings) {
  if (settings.p != m_pidSettings.p) {
    ConsoleLogger::getInstance().logInfo("PidMotorControllerPair",
                                         "Setting P to %.6f for %s", settings.p,
                                         m_shuffleboardName.c_str());
    m_controllerFirst.SetP(settings.p);
    m_controllerSecond.SetP(settings.p);
  }

  if (settings.i != m_pidSettings.i) {
    ConsoleLogger::getInstance().logInfo("PidMotorControllerPair",
                                         "Setting I to %.6f for %s", settings.i,
                                         m_shuffleboardName.c_str());
    m_controllerFirst.SetI(settings.i);
    m_controllerSecond.SetI(settings.i);
  }

  if (settings.d != m_pidSettings.d) {
    ConsoleLogger::getInstance().logInfo("PidMotorControllerPair",
                                         "Setting D to %.6f for %s", settings.d,
                                         m_shuffleboardName.c_str());
    m_controllerFirst.SetD(settings.d);
    m_controllerSecond.SetD(settings.d);
  }

  if (settings.iZone != m_pidSettings.iZone) {
    ConsoleLogger::getInstance().logInfo(
        "PidMotorControllerPair", "Setting IZone to %.6f for %s",
        settings.iZone, m_shuffleboardName.c_str());
    m_controllerFirst.SetIZone(settings.iZone);
    m_controllerSecond.SetIZone(settings.iZone);
  }

  if (settings.ff != m_pidSettings.ff) {
    ConsoleLogger::getInstance().logInfo(
        "PidMotorControllerPair", "Setting FF to %.6f for %s", settings.ff,
        m_shuffleboardName.c_str());
    m_controllerFirst.SetFF(settings.ff);
    m_controllerSecond.SetFF(settings.ff);
  }

  m_pidSettings = settings;
}

const PidSettings& PidMotorControllerPair::GetPidSettings() const {
  return m_pidSettings;
}

PidMotorControllerPairTuner::PidMotorControllerPairTuner(
    PidMotorControllerPair& controllerPair)
    : m_controllerPair{controllerPair} {
  frc::SmartDashboard::PutNumber(
      m_controllerPair.m_shuffleboardName + " P Gain",
      m_controllerPair.GetPidSettings().p);
  frc::SmartDashboard::PutNumber(
      m_controllerPair.m_shuffleboardName + " I Gain",
      m_controllerPair.GetPidSettings().i);
  frc::SmartDashboard::PutNumber(
      m_controllerPair.m_shuffleboardName + " D Gain",
      m_controllerPair.GetPidSettings().d);
  frc::SmartDashboard::PutNumber(m_controllerPair.m_shuffleboardName + " IZone",
                                 m_controllerPair.GetPidSettings().iZone);
  frc::SmartDashboard::PutNumber(
      m_controllerPair.m_shuffleboardName + " Feed Forward",
      m_controllerPair.GetPidSettings().ff);
}

void PidMotorControllerPairTuner::UpdateFromShuffleboard() {
  double tP = frc::SmartDashboard::GetNumber(
      m_controllerPair.m_shuffleboardName + " P Gain",
      m_controllerPair.GetPidSettings().p);
  double tI = frc::SmartDashboard::GetNumber(
      m_controllerPair.m_shuffleboardName + " I Gain",
      m_controllerPair.GetPidSettings().i);
  double tD = frc::SmartDashboard::GetNumber(
      m_controllerPair.m_shuffleboardName + " D Gain",
      m_controllerPair.GetPidSettings().d);
  double tIZone = frc::SmartDashboard::GetNumber(
      m_controllerPair.m_shuffleboardName + " IZone",
      m_controllerPair.GetPidSettings().iZone);
  double tFeedForward = frc::SmartDashboard::GetNumber(
      m_controllerPair.m_shuffleboardName + " Feed Forward",
      m_controllerPair.GetPidSettings().ff);

  m_controllerPair.UpdatePidSettings(
      {.p = tP, .i = tI, .d = tD, .iZone = tIZone, .ff = tFeedForward});
}