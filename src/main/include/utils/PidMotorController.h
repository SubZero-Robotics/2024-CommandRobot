#pragma once

#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <string>

#include "utils/ConsoleLogger.h"
#include "utils/ShuffleboardLogger.h"

struct PidSettings {
  double p, i, d, iZone, ff;
};

// TODO: Group into a single, combined typename
template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder>
class PidMotorController {
 public:
  /// @brief
  /// @param name Name of the PIDMotorController
  /// @param controller Controller
  /// @param encoder
  /// @param pidSettings The PidSettings
  /// @param maxRpm The maximum RPM of the motor
  explicit PidMotorController(std::string name, TMotor &motor,
                              TRelativeEncoder &encoder,
                              TController &controller, PidSettings pidSettings,
                              TAbsoluteEncoder *absEncoder,
                              units::revolutions_per_minute_t maxRpm)
      : m_name{name},
        m_motor{motor},
        m_controller{controller},
        m_encoder{encoder},
        m_absEncoder{absEncoder},
        m_settings{pidSettings},
        m_pidController{
            frc::PIDController{pidSettings.p, pidSettings.i, pidSettings.d}},
        m_maxRpm{maxRpm} {
    // Doing it here so the PID controllers themselves get updated
    UpdatePidSettings(pidSettings);
  }

  void Set(double percentage) { m_motor.Set(percentage); }

  void Set(units::volt_t volts) { m_motor.SetVoltage(volts); }

  void SetPidTolerance(double tolerance) {
    m_pidController.SetTolerance(tolerance);
  }

  /// @brief Call this every loop in Periodic

  void Update() {
    if (m_absolutePositionEnabled) {
      ConsoleLogger::getInstance().logVerbose(
          m_name,
          "relative position %0.3f, absolute position %0.3f, absolute target"
          "%0.3f",
          GetEncoderPosition(), GetAbsoluteEncoderPosition(), m_absoluteTarget);
      auto effort =
          m_pidController.Calculate(GetEncoderPosition(), m_absoluteTarget);
      double totalEffort = effort;
      Set(units::volt_t(totalEffort));

      if (m_pidController.AtSetpoint()) {
        m_pidController.Reset();
        m_absolutePositionEnabled = false;
        Stop();
      }
    }
  }

  /// @brief
  /// @param rpm The desired RPM
  void RunWithVelocity(units::revolutions_per_minute_t rpm) {
    m_absolutePositionEnabled = false;
    m_controller.SetReference(rpm.value(),
                              rev::CANSparkBase::ControlType::kVelocity);
  }

  /// @brief
  /// @param percentage
  void RunWithVelocity(double percentage) {
    if (abs(percentage) > 1.0) {
      ConsoleLogger::getInstance().logError(
          "PidMotorController",
          "Incorrect percentages for motor %s: Value=%.4f ", m_name.c_str(),
          percentage);
      return;
    }
    auto rpm = units::revolutions_per_minute_t(m_maxRpm) * percentage;

    RunWithVelocity(rpm);
  }

  void RunToPosition(double position) {
    ShuffleboardLogger::getInstance().logVerbose(m_name + " Target position",
                                                 position);
    Stop();
    m_pidController.Reset();
    m_absolutePositionEnabled = true;
    m_absoluteTarget = position;
  }

  virtual void ResetEncoder() {
    m_encoder.SetPosition(0);
    ConsoleLogger::getInstance().logInfo(m_name + " PID Controller",
                                         "Reset encoder%s", "");
  }

  double GetEncoderPosition() { return m_encoder.GetPosition(); }

  std::optional<double> GetAbsoluteEncoderPosition() {
    if (m_absEncoder) {
      return m_absEncoder->GetPosition();
    }

    return std::nullopt;
  }

  void SetEncoderConversionFactor(double factor) {
    m_encoder.SetPositionConversionFactor(factor);
  }

  void SetAbsoluteEncoderConversionFactor(double factor) {
    if (m_absEncoder) {
      m_absEncoder->SetPositionConversionFactor(factor);
    }
  }

  /// @brief Stop the motor
  // TODO: USE MOTOR HERE, THIS IS BAD AND SHOULD NOT BE EMPTY
  void Stop() {
    m_absolutePositionEnabled = false;
    m_motor.Set(0);
  }

  PidSettings GetPidSettings() { return m_settings; }

  void UpdatePidSettings(PidSettings settings) {
    if (settings.p != m_settings.p) {
      ConsoleLogger::getInstance().logInfo("PidMotorController",
                                           "Setting P to %.6f for %s",
                                           settings.p, m_name.c_str());
      m_controller.SetP(settings.p);
    }

    if (settings.i != m_settings.i) {
      ConsoleLogger::getInstance().logInfo("PidMotorController",
                                           "Setting I to %.6f for %s",
                                           settings.i, m_name.c_str());
      m_controller.SetI(settings.i);
    }

    if (settings.d != m_settings.d) {
      ConsoleLogger::getInstance().logInfo("PidMotorController",
                                           "Setting D to %.6f for %s",
                                           settings.d, m_name.c_str());
      m_controller.SetD(settings.d);
    }

    if (settings.iZone != m_settings.iZone) {
      ConsoleLogger::getInstance().logInfo("PidMotorController",
                                           "Setting IZone to %.6f for %s",
                                           settings.iZone, m_name.c_str());
      m_controller.SetIZone(settings.iZone);
    }

    if (settings.ff != m_settings.ff) {
      ConsoleLogger::getInstance().logInfo("PidMotorController",
                                           "Setting FF to %.6f for %s",
                                           settings.ff, m_name.c_str());
      m_controller.SetFF(settings.ff);
    }

    m_settings = settings;
  }

  const std::string m_name;

 protected:
  TMotor &m_motor;
  TController &m_controller;
  TRelativeEncoder &m_encoder;
  TAbsoluteEncoder *m_absEncoder;
  PidSettings m_settings;
  frc::PIDController m_pidController;
  bool m_absolutePositionEnabled = false;
  double m_absoluteTarget = 0;
  const units::revolutions_per_minute_t m_maxRpm;
};

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder>
class PidMotorControllerTuner {
 public:
  explicit PidMotorControllerTuner(
      PidMotorController<TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder> &controller)
      : m_controller{controller} {
    frc::SmartDashboard::PutNumber(m_controller.m_name + " P Gain",
                                   m_controller.GetPidSettings().p);
    frc::SmartDashboard::PutNumber(m_controller.m_name + " I Gain",
                                   m_controller.GetPidSettings().i);
    frc::SmartDashboard::PutNumber(m_controller.m_name + " D Gain",
                                   m_controller.GetPidSettings().d);
    frc::SmartDashboard::PutNumber(m_controller.m_name + " IZone",
                                   m_controller.GetPidSettings().iZone);
    frc::SmartDashboard::PutNumber(m_controller.m_name + " Feed Forward",
                                   m_controller.GetPidSettings().ff);
  }
  /// @brief Call this within the Periodic method of the encapsulating subsystem
  void UpdateFromShuffleboard() {
    double tP = frc::SmartDashboard::GetNumber(m_controller.m_name + " P Gain",
                                               m_controller.GetPidSettings().p);
    double tI = frc::SmartDashboard::GetNumber(m_controller.m_name + " I Gain",
                                               m_controller.GetPidSettings().i);
    double tD = frc::SmartDashboard::GetNumber(m_controller.m_name + " D Gain",
                                               m_controller.GetPidSettings().d);
    double tIZone = frc::SmartDashboard::GetNumber(
        m_controller.m_name + " IZone", m_controller.GetPidSettings().iZone);
    double tFeedForward =
        frc::SmartDashboard::GetNumber(m_controller.m_name + " Feed Forward",
                                       m_controller.GetPidSettings().ff);

    m_controller.UpdatePidSettings(
        {.p = tP, .i = tI, .d = tD, .iZone = tIZone, .ff = tFeedForward});
  }

 private:
  PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder>
      &m_controller;
};

class RevPidMotorController
    : public PidMotorController<rev::CANSparkMax, rev::SparkPIDController,
                                rev::SparkRelativeEncoder,
                                rev::SparkAbsoluteEncoder> {};