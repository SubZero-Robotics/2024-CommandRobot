#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include "utils/ConsoleLogger.h"

struct PidSettings {
  double p, i, d, iZone, ff;
};

class PidMotorControllerPair {
 public:
  /// @brief
  /// @param name Name of the pair
  /// @param controllerFirst First controller
  /// @param controllerSecond Second controller
  /// @param pidSettings
  /// @param maxRpm The maximum RPM of the motors
  explicit PidMotorControllerPair(std::string, rev::SparkPIDController &,
                                  rev::SparkPIDController &, PidSettings,
                                  units::revolutions_per_minute_t);

  /// @brief
  /// @param rpmFirst RPM of the first motor
  /// @param rpmSecond RPM of the second motor
  void RunWithVelocity(units::revolutions_per_minute_t,
                       units::revolutions_per_minute_t);

  /// @brief
  /// @param percentageFirst Percentage of the max RPM of the first motor
  /// @param percentageSecond Percentage of the max RPM of the second motor
  void RunWithVelocity(double, double);

  /// @brief Stop both motors
  void Stop();

  const PidSettings &GetPidSettings() const;

  void UpdatePidSettings(PidSettings);

  const std::string m_shuffleboardName;

 private:
  rev::SparkPIDController &m_controllerFirst;
  rev::SparkPIDController &m_controllerSecond;
  PidSettings m_pidSettings;
  const units::revolutions_per_minute_t m_maxRpm;
};

class PidMotorControllerPairTuner {
 public:
  explicit PidMotorControllerPairTuner(PidMotorControllerPair &);

  /// @brief Call this within the Periodic method of the encapsulating subsystem
  void UpdateFromShuffleboard();

 private:
  PidMotorControllerPair &m_controllerPair;
};