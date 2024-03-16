#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include "utils/ConsoleLogger.h"
#include "utils/PidMotorController.h"

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder>
class PidMotorControllerPair {
 public:
  /// @brief
  /// @param prefix Name prefix in shuffleboard
  /// @param first First controller
  /// @param second Second controller
  explicit PidMotorControllerPair(
      std::string prefix,
      PidMotorController<TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder> &first,
      PidMotorController<TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder> &second)
      : m_shuffleboardPrefix{prefix},
        m_controllerFirst{first},
        m_controllerSecond{second} {}

  /// @brief
  /// @param rpmFirst RPM of the first motor
  /// @param rpmSecond RPM of the second motor
  void RunWithVelocity(units::revolutions_per_minute_t rpmFirst,
                       units::revolutions_per_minute_t rpmSecond) {
    m_controllerFirst.RunWithVelocity(rpmFirst);
    m_controllerSecond.RunWithVelocity(rpmSecond);
  }

  /// @brief
  /// @param percentageFirst Percentage of the max RPM of the first motor
  /// @param percentageSecond Percentage of the max RPM of the second motor
  void RunWithVelocity(double percentageFirst, double percentageSecond) {
    m_controllerFirst.RunWithVelocity(percentageFirst);
    m_controllerSecond.RunWithVelocity(percentageSecond);
  }

  /// @brief Stop both motors
  void Stop() {
    m_controllerFirst.Stop();
    m_controllerSecond.Stop();
  }

  const PidSettings &GetPidSettings() const { return m_pidSettings; }

  void UpdatePidSettings(PidSettings settings) {
    m_controllerFirst.UpdatePidSettings(settings);
    m_controllerSecond.UpdatePidSettings(settings);

    m_pidSettings = settings;
  }

  const std::string m_shuffleboardPrefix;

 private:
  PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder>
      &m_controllerFirst;
  PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder>
      &m_controllerSecond;
  PidSettings m_pidSettings;
};

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder>
class PidMotorControllerPairTuner {
 public:
  explicit PidMotorControllerPairTuner(
      PidMotorControllerPair<TMotor, TController, TRelativeEncoder,
                             TAbsoluteEncoder> &controllerPair)
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

  /// @brief Call this within the Periodic method of the encapsulating subsystem
  void UpdateFromShuffleboard() {
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

 private:
  PidMotorControllerPair<TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder> &m_controllerPair;
};