#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include "utils/ConsoleLogger.h"

struct PidSettings {
  double p, i, d, iZone, ff;
};

template <typename TController, typename TEncoder>
class PidMotorController {
 public:
  /// @brief
  /// @param name Name of the PIDMotorController
  /// @param controller Controller
  /// @param encoder
  /// @param pidSettings The PidSettings
  /// @param maxRpm The maximum RPM of the motor
  explicit PidMotorController(std::string name, TController &controller,
                              TEncoder &encoder, PidSettings pidSettings,
                              units::revolutions_per_minute_t maxRpm)
      : m_shuffleboardName{name},
        m_controller{controller},
        m_encoder{encoder},
        m_settings{pidSettings},
        m_maxRpm{maxRpm} {
    // Doing it here so the PID controllers themselves get updated
    UpdatePidSettings(pidSettings);
  }

  /// @brief
  /// @param rpm The desired RPM
  void RunWithVelocity(units::revolutions_per_minute_t rpm) {
    m_controller.SetReference(rpm.value(),
                              rev::CANSparkBase::ControlType::kVelocity);
  }

  /// @brief
  /// @param percentage
  void RunWithVelocity(double percentage) {
    if (abs(percentage) > 1.0) {
      ConsoleLogger::getInstance().logError(
          "PidMotorController",
          "Incorrect percentages for motor %s: Value=%.4f ",
          m_shuffleboardName.c_str(), percentage);
      return;
    }
    auto rpm = units::revolutions_per_minute_t(m_maxRpm) * percentage;

    RunWithVelocity(rpm);
  }

  double GetEncoderPosition() {
    // TODO:
    return 0;
  }

  /// @brief Stop the motor
  void Stop() { RunWithVelocity(0); }

  const PidSettings &GetPidSettings() const { return m_settings; }

  void UpdatePidSettings(PidSettings settings) {
    if (settings.p != m_settings.p) {
      ConsoleLogger::getInstance().logInfo(
          "PidMotorController", "Setting P to %.6f for %s", settings.p,
          m_shuffleboardName.c_str());
      m_controller.SetP(settings.p);
    }

    if (settings.i != m_settings.i) {
      ConsoleLogger::getInstance().logInfo(
          "PidMotorController", "Setting I to %.6f for %s", settings.i,
          m_shuffleboardName.c_str());
      m_controller.SetI(settings.i);
    }

    if (settings.d != m_settings.d) {
      ConsoleLogger::getInstance().logInfo(
          "PidMotorController", "Setting D to %.6f for %s", settings.d,
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

  const std::string m_shuffleboardName;

 private:
  TController &m_controller;
  TEncoder &m_encoder;
  PidSettings m_settings;
  const units::revolutions_per_minute_t m_maxRpm;
};

template <typename TController, typename TEncoder>
class PidMotorControllerTuner {
 public:
  explicit PidMotorControllerTuner(
      PidMotorController<TController, TEncoder> &controller)
      : m_controller{controller} {
    frc::SmartDashboard::PutNumber(m_controller.m_shuffleboardName + " P Gain",
                                   m_controller.GetPidSettings().p);
    frc::SmartDashboard::PutNumber(m_controller.m_shuffleboardName + " I Gain",
                                   m_controller.GetPidSettings().i);
    frc::SmartDashboard::PutNumber(m_controller.m_shuffleboardName + " D Gain",
                                   m_controller.GetPidSettings().d);
    frc::SmartDashboard::PutNumber(m_controller.m_shuffleboardName + " IZone",
                                   m_controller.GetPidSettings().iZone);
    frc::SmartDashboard::PutNumber(
        m_controller.m_shuffleboardName + " Feed Forward",
        m_controller.GetPidSettings().ff);
  }
  /// @brief Call this within the Periodic method of the encapsulating subsystem
  void UpdateFromShuffleboard() {
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

 private:
  PidMotorController<TController, TEncoder> &m_controller;
};

template <typename TController, typename TEncoder>
class RevPidMotorController
    : public PidMotorController<rev::SparkPIDController, rev::RelativeEncoder> {
};