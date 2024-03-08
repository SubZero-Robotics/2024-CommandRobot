#include "utils/PidMotorControllerPair.h"

PidMotorControllerPair::PidMotorControllerPair(std::string prefix,
                                               PidMotorController& first,
                                               PidMotorController& second)
    : m_shuffleboardPrefix{prefix},
      m_controllerFirst{first},
      m_controllerSecond{second} {}

void PidMotorControllerPair::RunWithVelocity(
    units::revolutions_per_minute_t rpmFirst,
    units::revolutions_per_minute_t rpmSecond) {
  m_controllerFirst.RunWithVelocity(rpmFirst);
  m_controllerSecond.RunWithVelocity(rpmSecond);
}

void PidMotorControllerPair::RunWithVelocity(double percentageFirst,
                                             double percentageSecond) {
  m_controllerFirst.RunWithVelocity(percentageFirst);
  m_controllerSecond.RunWithVelocity(percentageSecond);
}

void PidMotorControllerPair::Stop() {
  m_controllerFirst.Stop();
  m_controllerSecond.Stop();
}

void PidMotorControllerPair::UpdatePidSettings(PidSettings settings) {
  m_controllerFirst.UpdatePidSettings(settings);
  m_controllerSecond.UpdatePidSettings(settings);

  m_pidSettings = settings;
}

const PidSettings& PidMotorControllerPair::GetPidSettings() const {
  return m_pidSettings;
}

PidMotorControllerPairTuner::PidMotorControllerPairTuner(
    PidMotorControllerPair& controllerPair)
    : m_controllerPair{controllerPair} {
  frc::SmartDashboard::PutNumber(
      m_controllerPair.m_shuffleboardPrefix + " P Gain",
      m_controllerPair.GetPidSettings().p);
  frc::SmartDashboard::PutNumber(
      m_controllerPair.m_shuffleboardPrefix + " I Gain",
      m_controllerPair.GetPidSettings().i);
  frc::SmartDashboard::PutNumber(
      m_controllerPair.m_shuffleboardPrefix + " D Gain",
      m_controllerPair.GetPidSettings().d);
  frc::SmartDashboard::PutNumber(
      m_controllerPair.m_shuffleboardPrefix + " IZone",
      m_controllerPair.GetPidSettings().iZone);
  frc::SmartDashboard::PutNumber(
      m_controllerPair.m_shuffleboardPrefix + " Feed Forward",
      m_controllerPair.GetPidSettings().ff);
}

void PidMotorControllerPairTuner::UpdateFromShuffleboard() {
  double tP = frc::SmartDashboard::GetNumber(
      m_controllerPair.m_shuffleboardPrefix + " P Gain",
      m_controllerPair.GetPidSettings().p);
  double tI = frc::SmartDashboard::GetNumber(
      m_controllerPair.m_shuffleboardPrefix + " I Gain",
      m_controllerPair.GetPidSettings().i);
  double tD = frc::SmartDashboard::GetNumber(
      m_controllerPair.m_shuffleboardPrefix + " D Gain",
      m_controllerPair.GetPidSettings().d);
  double tIZone = frc::SmartDashboard::GetNumber(
      m_controllerPair.m_shuffleboardPrefix + " IZone",
      m_controllerPair.GetPidSettings().iZone);
  double tFeedForward = frc::SmartDashboard::GetNumber(
      m_controllerPair.m_shuffleboardPrefix + " Feed Forward",
      m_controllerPair.GetPidSettings().ff);

  m_controllerPair.UpdatePidSettings(
      {.p = tP, .i = tI, .d = tD, .iZone = tIZone, .ff = tFeedForward});
}