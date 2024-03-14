#pragma once

#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include "utils/ConsoleLogger.h"

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
                              PidSettings pidSettings,
                              std::optional<TAbsoluteEncoder *> absEncoder,
                              units::revolutions_per_minute_t maxRpm)
      : m_shuffleboardName{name},
        m_motor{motor},
        m_controller{motor.GetPIDController()},
        m_encoder{encoder},
        m_settings{pidSettings},
        m_pidController{
            frc::PIDController{pidSettings.p, pidSettings.i, pidSettings.d}},
        m_maxRpm{maxRpm} {
    // Doing it here so the PID controllers themselves get updated
    UpdatePidSettings(pidSettings);

    // TODO: constant
    m_pidController.SetTolerance(1.5);
  }

  void Set(double percentage) { m_motor.Set(percentage); }

  void Set(units::volt_t volts) { m_motor.SetVoltage(volts); }

  /// @brief Call this every loop in Periodic
  void Update() {
    if (m_absolutePositionEnabled) {
      auto effort =
          m_pidController.Calculate(GetEncoderPosition(), m_absoluteTarget);
      double totalEffort = ffEffort + effort;
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
          "Incorrect percentages for motor %s: Value=%.4f ",
          m_shuffleboardName.c_str(), percentage);
      return;
    }
    auto rpm = units::revolutions_per_minute_t(m_maxRpm) * percentage;

    RunWithVelocity(rpm);
  }

  void RunToPosition(double rotations) {
    ConsoleLogger::getInstance().logVerbose(
        m_shuffleboardName, "Setting rotations %0.3f", rotations);
    Stop();
    m_absolutePositionEnabled = true;
    m_absoluteTarget = rotations;
    // m_controller.SetReference(rotations,
    //                           rev::CANSparkBase::ControlType::kPosition);
  }

  virtual void ResetEncoder() { m_encoder.SetPosition(0); }

  double GetEncoderPosition() { return m_encoder.GetPosition(); }

  std::optional<double> GetAbsoluteEncoderPosition() {
    if (m_absEncoder.has_value() && m_absEncoder.value()) {
      return m_absEncoder.value()->GetPosition();
    }

    return std::nullopt;
  }

  /// @brief Stop the motor
  // TODO: USE MOTOR HERE, THIS IS BAD AND SHOULD NOT BE EMPTY
  void Stop() {
    m_absolutePositionEnabled = false;
    m_motor.Set(0);
  }

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

 protected:
  TMotor &m_motor;
  TController &m_controller;
  TRelativeEncoder &m_encoder;
  std::optional<TAbsoluteEncoder *> m_absEncoder;
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
  PidMotorController<typename TMotor, typename TController,
                     typename TRelativeEncoder, typename TAbsoluteEncoder>
      &m_controller;
};

class RevPidMotorController
    : public PidMotorController<rev::CANSparkMax, rev::SparkPIDController,
                                rev::SparkRelativeEncoder,
                                rev::SparkAbsoluteEncoder> {};