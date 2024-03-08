#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

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
  explicit PidMotorController(std::string, TController &, TEncoder &,
                              PidSettings, units::revolutions_per_minute_t);

  /// @brief
  /// @param rpm The desired RPM
  void RunWithVelocity(units::revolutions_per_minute_t);

  /// @brief
  /// @param percentage
  void RunWithVelocity(double);

  double GetEncoderPosition();

  /// @brief Stop the motor
  void Stop();

  const PidSettings &GetPidSettings() const;

  void UpdatePidSettings(PidSettings);

  const std::string m_shuffleboardName;

 private:
  TController &m_controller;
  TEncoder &m_encoder;
  PidSettings m_settings;
  const units::revolutions_per_minute_t m_maxRpm;
};

template <template <typename TController, typename TEncoder> class Controller>
class PidMotorControllerTuner {
 public:
  explicit PidMotorControllerTuner(Controller &);
  /// @brief Call this within the Periodic method of the encapsulating subsystem
  void UpdateFromShuffleboard();

 private:
  Controller &m_controller;
};

template <typename TController, typename TEncoder>
class RevPidMotorController
    : public PidMotorController<rev::SparkPIDController, rev::RelativeEncoder> {
};