#include "utils/PidMotorController.h"

#include "utils/ConsoleLogger.h"

PidMotorController::PidMotorController(std::string name,
                                       rev::SparkPIDController& controller,
                                       PidSettings pidSettings,
                                       units::revolutions_per_minute_t rpm)
    : m_shuffleboardName{name},
      m_controller{controller},
      m_settings{pidSettings},
      m_maxRpm{rpm} {
  // Doing it here so the PID controllers themselves get updated
  UpdatePidSettings(pidSettings);
}

void PidMotorController::RunWithVelocity(units::revolutions_per_minute_t rpm) {
  m_controller.SetReference(rpm.value(),
                            rev::CANSparkBase::ControlType::kVelocity);
}

void PidMotorController::RunWithVelocity(double percentage) {
  if (abs(percentage) > 1.0) {
    ConsoleLogger::getInstance().logError(
        "PidMotorController", "Incorrect percentages for motor %s: Value=%.4f ",
        m_shuffleboardName.c_str(), percentage);
    return;
  }
  auto rpm = units::revolutions_per_minute_t(m_maxRpm) * percentage;

  RunWithVelocity(rpm);
}

void PidMotorController::Stop() { RunWithVelocity(0); }

void PidMotorController::UpdatePidSettings(PidSettings settings) {
  if (settings.p != m_settings.p) {
    ConsoleLogger::getInstance().logInfo("PidMotorController",
                                         "Setting P to %.6f for %s", settings.p,
                                         m_shuffleboardName.c_str());
    m_controller.SetP(settings.p);
  }

  if (settings.i != m_settings.i) {
    ConsoleLogger::getInstance().logInfo("PidMotorController",
                                         "Setting I to %.6f for %s", settings.i,
                                         m_shuffleboardName.c_str());
    m_controller.SetI(settings.i);
  }

  if (settings.d != m_settings.d) {
    ConsoleLogger::getInstance().logInfo("PidMotorController",
                                         "Setting D to %.6f for %s", settings.d,
                                         m_shuffleboardName.c_str());
    m_controller.SetD(settings.d);
  }

  if (settings.iZone != m_settings.iZone) {
    ConsoleLogger::getInstance().logInfo(
        "PidMotorController", "Setting IZone to %.6f for %s", settings.iZone,
        m_shuffleboardName.c_str());
    m_controller.SetIZone(settings.iZone);
  }

  if (settings.ff != m_settings.ff) {
    ConsoleLogger::getInstance().logInfo(
        "PidMotorController", "Setting FF to %.6f for %s", settings.ff,
        m_shuffleboardName.c_str());
    m_controller.SetFF(settings.ff);
  }

  m_settings = settings;
}

const PidSettings& PidMotorController::GetPidSettings() const {
  return m_settings;

}

PidMotorControllerTuner::PidMotorControllerTuner(
    PidMotorController& controllerPair)
    : m_controller{controllerPair} {
  frc::SmartDashboard::PutNumber(
      m_controller.m_shuffleboardName + " P Gain",
      m_controller.GetPidSettings().p);
  frc::SmartDashboard::PutNumber(
      m_controller.m_shuffleboardName + " I Gain",
      m_controller.GetPidSettings().i);
  frc::SmartDashboard::PutNumber(
      m_controller.m_shuffleboardName + " D Gain",
      m_controller.GetPidSettings().d);
  frc::SmartDashboard::PutNumber(
      m_controller.m_shuffleboardName + " IZone",
      m_controller.GetPidSettings().iZone);
  frc::SmartDashboard::PutNumber(
      m_controller.m_shuffleboardName + " Feed Forward",
      m_controller.GetPidSettings().ff);
}

void PidMotorControllerTuner::UpdateFromShuffleboard() {
  double tP = frc::SmartDashboard::GetNumber(
      m_controller.m_shuffleboardName + " P Gain",
      m_controller.GetPidSettings().p);
  double tI = frc::SmartDashboard::GetNumber(
      m_controller.m_shuffleboardName + " I Gain",
      m_controller.GetPidSettings().i);
  double tD = frc::SmartDashboard::GetNumber(
      m_controller.m_shuffleboardName + " D Gain",
      m_controller.GetPidSettings().d);
  double tIZone = frc::SmartDashboard::GetNumber(
      m_controller.m_shuffleboardName + " IZone",
      m_controller.GetPidSettings().iZone);
  double tFeedForward = frc::SmartDashboard::GetNumber(
      m_controller.m_shuffleboardName + " Feed Forward",
      m_controller.GetPidSettings().ff);

  m_controller.UpdatePidSettings(
      {.p = tP, .i = tI, .d = tD, .iZone = tIZone, .ff = tFeedForward});
}
