#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include "utils/ConsoleLogger.h"
#include "utils/PidMotorController.h"

class PidMotorControllerPair {
 public:
  /// @brief
  /// @param prefix Name prefix in shuffleboard
  /// @param first First controller
  /// @param second Second controller
  explicit PidMotorControllerPair(std::string, PidMotorController &,
                                  PidMotorController &);

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

  const std::string m_shuffleboardPrefix;

 private:
  PidMotorController &m_controllerFirst;
  PidMotorController &m_controllerSecond;
  PidSettings m_pidSettings;
};

class PidMotorControllerPairTuner {
 public:
  explicit PidMotorControllerPairTuner(PidMotorControllerPair &);

  /// @brief Call this within the Periodic method of the encapsulating subsystem
  void UpdateFromShuffleboard();

 private:
  PidMotorControllerPair &m_controllerPair;
};