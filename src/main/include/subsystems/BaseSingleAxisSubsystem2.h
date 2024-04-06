#pragma once

#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/TrapezoidProfileSubsystem.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <memory>
#include <string>

#include "Constants.h"
#include "subsystems/ISingleAxisSubsystem.h"
#include "utils/ConsoleLogger.h"
#include "utils/PidMotorController.h"
#include "utils/ShuffleboardLogger.h"

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TDistance>
class BaseSingleAxisSubsystem2
    : public ISingleAxisSubsystem2<TDistance>,
      public frc2::TrapezoidProfileSubsystem<TDistance> {
 public:
  using PidState = typename frc::TrapezoidProfile<TDistance>::State;
  using Distance_t = units::unit_t<TDistance>;
  using Velocity =
      units::compound_unit<TDistance, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<Velocity>;
  using Acceleration =
      units::compound_unit<Velocity, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<Acceleration>;

 protected:
  bool IsMovementAllowed(double speed, bool ignoreEncoder = false) {
    bool atMin = ignoreEncoder ? AtLimitSwitchMin() : AtHome();
    bool atMax = ignoreEncoder ? AtLimitSwitchMax() : AtMax();

    if (atMin) {
      // ConsoleLogger::getInstance().logVerbose(
      //     m_name, "At minimum; movement allowed = %d", speed >= 0);
      return speed >= 0;
    }

    if (atMax) {
      ConsoleLogger::getInstance().logVerbose(
          m_name, "At maximum, movement allowed = %d", speed <= 0);
      return speed <= 0;
    }

    return true;
  }

  bool IsMovementAllowed(bool ignoreEncoder = false) {
    bool atMin = ignoreEncoder ? AtLimitSwitchMin() : AtHome();
    bool atMax = ignoreEncoder ? AtLimitSwitchMax() : AtMax();

    if (atMin) {
      return false;
    }

    if (atMax) {
      return false;
    }

    return true;
  }

 public:
  BaseSingleAxisSubsystem2(
      std::string name,
      PidMotorController<TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder> &controller,
      ISingleAxisSubsystem2<TDistance>::SingleAxisConfig2 config)
      : frc2::TrapezoidProfileSubsystem<
            TDistance>{AutoConstants::kSingleAxisConstraints},
        m_minLimitSwitch{config.minLimitSwitch},
        m_maxLimitSwitch{config.maxLimitSwitch},
        m_controller{controller},
        m_config{config},
        m_name{name},
        m_pidEnabled{false} {
    m_pidEnabled = false;

    frc2::TrapezoidProfileSubsystem<TDistance>::Disable();
  }

  void Periodic() override {
    if (m_controller.GetAbsoluteEncoderPosition().has_value()) {
      Distance_t absEncValue = Distance_t(
          std::abs(m_controller.GetAbsoluteEncoderPosition().value()));

      // ConsoleLogger::getInstance().logVerbose(
      //     m_name, "Absolute encoder=%0.3f | Relative encoder=%0.3f",
      //     m_controller.GetAbsoluteEncoderPosition().value(),
      //     m_controller.GetEncoderPosition());

      if (!resetOccurred && absEncValue <= m_config.tolerance) {
        m_controller.ResetEncoder();
        resetOccurred = true;
      } else if (resetOccurred && absEncValue > m_config.tolerance) {
        resetOccurred = false;
      }
    }

    if (m_pidEnabled) {
      frc2::TrapezoidProfileSubsystem<TDistance>::Periodic();
      m_controller.Update();
    }

    if (!m_pidEnabled && !IsMovementAllowed(m_latestSpeed)) {
      ConsoleLogger::getInstance().logInfo(
          m_name, "Periodic: Movement with speed %f is not allowed",
          m_latestSpeed);

      Stop();
    }
  }

  virtual void RunMotorVelocity(Velocity_t speed,
                                bool ignoreEncoder = false) = 0;

  void UseState(PidState setpoint) override {
    // if (IsMovementAllowed()) {
    m_controller.RunToPosition(setpoint.position.value());
    // } else {
    //   ConsoleLogger::getInstance().logVerbose(
    //       m_name, "PID Trying to move out of limits %s", "");
    //   Stop();
    // }
  }

  void RunMotorSpeedDefault(bool ignoreEncoder = false) override {
    RunMotorVelocity(m_config.defaultSpeed, ignoreEncoder);
  }

  void RunMotorPercentage(double percentSpeed,
                          bool ignoreEncoder = false) override {
    if (!IsMovementAllowed(percentSpeed, ignoreEncoder)) {
      ConsoleLogger::getInstance().logInfo(
          m_name, "Movement with speed %f is not allowed", percentSpeed);
      return;
    }

    ConsoleLogger::getInstance().logVerbose(m_name, "Running with speed = %f",
                                            percentSpeed);
    DisablePid();
    m_controller.RunWithVelocity(percentSpeed);
  }

  Distance_t GetCurrentPosition() override {
    return Distance_t(m_controller.GetEncoderPosition());
  }

  void Stop() override {
    ConsoleLogger::getInstance().logInfo(m_name, "Stopping%s", "");
    m_controller.Stop();
  }

  void ResetEncoder() override {
    ConsoleLogger::getInstance().logInfo(m_name, "Reset encoder%s", "");
    m_controller.ResetEncoder();
  }

  bool AtHome() override {
    ShuffleboardLogger::getInstance().logVerbose(
        m_name, "Current Position %0.3f, Min Distance %0.3f",
        GetCurrentPosition().value(), m_config.minDistance.value());
    return AtLimitSwitchMin() || GetCurrentPosition() <= m_config.minDistance;
  }

  bool AtMax() override {
    return AtLimitSwitchMax() || GetCurrentPosition() >= m_config.maxDistance;
  }

  bool AtLimitSwitchMin() override {
    if (m_minLimitSwitch && m_minLimitSwitch.value()) {
      return !m_minLimitSwitch.value()->Get();
    }

    return false;
  }

  bool AtLimitSwitchMax() override {
    if (m_maxLimitSwitch && m_maxLimitSwitch.value()) {
      return !m_maxLimitSwitch.value()->Get();
    }

    return false;
  }

  frc2::CommandPtr MoveToPositionAbsolute(Distance_t position) override {
    return frc2::InstantCommand(
               [this, position] {
                 if (position < m_config.minDistance ||
                     position > m_config.maxDistance) {
                   ConsoleLogger::getInstance().logWarning(
                       m_name,
                       "Attempting to move to position %f outside of boundary",
                       position.value());
                   return;
                 }

                 ConsoleLogger::getInstance().logVerbose(
                     m_name, "Moving to absolute position %f",
                     position.value());

                 m_goalPosition = position;
                 EnablePid();
                 frc2::TrapezoidProfileSubsystem<TDistance>::SetGoal(position);
               },
               {this})
        .ToPtr();
  }

  frc2::CommandPtr MoveToPositionRelative(Distance_t position) override {
    return MoveToPositionAbsolute(m_goalPosition + position);
  }

  frc2::CommandPtr Home() override {
    return frc2::FunctionalCommand(
               // OnInit
               [this] { Stop(); },
               // OnExecute
               [this] { RunMotorSpeedDefault(true); },
               // OnEnd
               [this](bool interrupted) {
                 Stop();
                 ResetEncoder();
               },
               // IsFinished
               [this] { return AtLimitSwitchMin(); }, {this})
        .ToPtr();
  }

  bool IsEnabled() override { return m_pidEnabled; }

  void DisablePid() override {
    m_pidEnabled = false;
    Stop();
    frc2::TrapezoidProfileSubsystem<TDistance>::Disable();
  }

  void EnablePid() override {
    m_pidEnabled = true;
    frc2::TrapezoidProfileSubsystem<TDistance>::Enable();
  }

  void OnInit() {
    m_controller.SetPidTolerance(m_config.tolerance.value());
    m_controller.SetEncoderConversionFactor(
        m_config.encoderDistancePerRevolution.value());

    if (m_config.absoluteEncoderDistancePerRevolution.has_value())
      m_controller.SetAbsoluteEncoderConversionFactor(
          m_config.absoluteEncoderDistancePerRevolution.value().value());
  }

 protected:
  std::optional<frc::DigitalInput *> m_minLimitSwitch;
  std::optional<frc::DigitalInput *> m_maxLimitSwitch;
  PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder>
      &m_controller;
  ISingleAxisSubsystem2<TDistance>::SingleAxisConfig2 m_config;
  std::string m_name;
  Distance_t m_goalPosition;
  bool m_pidEnabled;
  bool m_home;
  bool resetOccurred = false;
  double m_latestSpeed;
};

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder>
class RotationalSingleAxisSubsystem
    : public BaseSingleAxisSubsystem2<TMotor, TController, TRelativeEncoder,
                                      TAbsoluteEncoder, units::degree> {
 public:
  RotationalSingleAxisSubsystem(
      std::string name,
      PidMotorController<TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder> &controller,
      ISingleAxisSubsystem2<units::degree>::SingleAxisConfig2 config,
      units::meter_t armatureLength)
      : BaseSingleAxisSubsystem2<TMotor, TController, TRelativeEncoder,
                                 TAbsoluteEncoder, units::degree>{name,
                                                                  controller,
                                                                  config},
        m_armatureLength{armatureLength} {}

  void RunMotorVelocity(units::degrees_per_second_t speed,
                        bool ignoreEncoder = false) override {
    if (!BaseSingleAxisSubsystem2<
            TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
            units::degree>::IsMovementAllowed(speed.value(), ignoreEncoder)) {
      return;
    }

    BaseSingleAxisSubsystem2<TMotor, TController, TRelativeEncoder,
                             TAbsoluteEncoder, units::degree>::DisablePid();

    BaseSingleAxisSubsystem2<TMotor, TController, TRelativeEncoder,
                             TAbsoluteEncoder, units::degree>::m_controller
        .RunWithVelocity(speed);
  }

 protected:
  units::meter_t m_armatureLength;
};

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder>
class LinearSingleAxisSubsystem
    : public BaseSingleAxisSubsystem2<TMotor, TController, TRelativeEncoder,
                                      TAbsoluteEncoder, units::meter> {
 public:
  LinearSingleAxisSubsystem(
      std::string name,
      PidMotorController<TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder> &controller,
      ISingleAxisSubsystem2<units::degree>::SingleAxisConfig2 config)
      : BaseSingleAxisSubsystem2<TMotor, TController, TRelativeEncoder,
                                 TAbsoluteEncoder, units::meter>{
            name, controller, config} {}

  void RunMotorVelocity(units::meters_per_second_t speed,
                        bool ignoreEncoder = false) override {
    BaseSingleAxisSubsystem2<TMotor, TController, TRelativeEncoder,
                             TAbsoluteEncoder, units::meter>::DisablePid();
    ConsoleLogger::getInstance().logWarning(
        BaseSingleAxisSubsystem2<TMotor, TController, TRelativeEncoder,
                                 TAbsoluteEncoder, units::meter>::m_name,
        "Running with a velocity is not supported for linear subsystems!%s",
        "");
  }
};