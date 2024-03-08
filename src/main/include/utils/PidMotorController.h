#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

struct PidSettings {
  double p, i, d, iZone, ff;
};

class PidMotorController {
 public:
  /// @brief
  /// @param name Name of the PIDMotorController
  /// @param controller Controller
  /// @param pidSettings The PidSettings
  /// @param maxRpm The maximum RPM of the motor
  explicit PidMotorController(std::string, rev::SparkPIDController &,
                              PidSettings, units::revolutions_per_minute_t);

  /// @brief
  /// @param rpm The desired RPM
  void RunWithVelocity(units::revolutions_per_minute_t);

  /// @brief
  /// @param percentage
  void RunWithVelocity(double);

  /// @brief Stop the motor
  void Stop();

  const PidSettings &GetPidSettings() const;

  void UpdatePidSettings(PidSettings);

  const std::string m_shuffleboardName;

 private:
  rev::SparkPIDController &m_controller;
  PidSettings m_settings;
  const units::revolutions_per_minute_t m_maxRpm;
};

class PidMotorControllerTuner {
 public:
  explicit PidMotorControllerTuner(PidMotorController &);
  /// @brief Call this within the Periodic method of the encapsulating subsystem
  void UpdateFromShuffleboard();

 private:
  PidMotorController &m_controller;
};