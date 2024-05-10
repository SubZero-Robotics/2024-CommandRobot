#pragma once

#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
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
#include "subsystems/singleaxis/ISingleAxisSubsystem.h"
#include "utils/ConsoleLogger.h"
#include "utils/PidMotorController.h"
#include "utils/ShuffleboardLogger.h"

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TDistance>
class BaseSingleAxisSubsystem
    : public ISingleAxisSubsystem<TDistance>,
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
    if (m_config.ignoreLimit()) return true;

    ConsoleVerbose("BaseSingleAxis", "Did not ignore limit%s", "");

    bool atMin = ignoreEncoder ? AtLimitSwitchMin() : AtHome();
    bool atMax = ignoreEncoder ? AtLimitSwitchMax() : AtMax();

    frc::SmartDashboard::PutBoolean(m_name + " At Min", atMin);
    frc::SmartDashboard::PutBoolean(m_name + " At Max", atMax);

    if (atMin) {
      return speed >= 0;
    }

    if (atMax) {
      return speed <= 0;
    }

    return true;
  }

  bool IsMovementAllowed(bool ignoreEncoder = false) {
    if (m_config.ignoreLimit()) return true;

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
  BaseSingleAxisSubsystem(
      std::string name,
      PidMotorController<TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder> &controller,
      ISingleAxisSubsystem<TDistance>::SingleAxisConfig config,
      frc::TrapezoidProfile<TDistance>::Constraints profileConstraints,
      frc::MechanismObject2d *mechanismNode = nullptr)
      : frc2::TrapezoidProfileSubsystem<TDistance>{profileConstraints},
        m_minLimitSwitch{config.minLimitSwitch},
        m_maxLimitSwitch{config.maxLimitSwitch},
        m_controller{controller},
        m_config{config},
        m_name{name},
        m_pidEnabled{false} {
    m_pidEnabled = false;

    if (mechanismNode) {
      m_ligament2d = mechanismNode->Append<frc::MechanismLigament2d>(
          name, config.mechanismConfig.minimumLength.value(),
          config.mechanismConfig.minimumAngle, config.mechanismConfig.lineWidth,
          config.mechanismConfig.color);
    }

    m_resetEncCmd = ResetRelativeEncoder();
    frc::SmartDashboard::PutData(m_name + " Reset Encoder",
                                 m_resetEncCmd.get());
    frc2::TrapezoidProfileSubsystem<TDistance>::Disable();
  }

  void Periodic() override {
    frc::SmartDashboard::PutBoolean(m_name + " Pid Enabled", m_pidEnabled);
    frc::SmartDashboard::PutNumber(m_name + " Position",
                                   GetCurrentPosition().value());
    if (m_config.conversionFunction) {
      std::string curPos =
          m_config.conversionFunction.value()(GetCurrentPosition());
      frc::SmartDashboard::PutString(m_name + " Converted Position", curPos);
    }

    if (m_controller.GetAbsoluteEncoderPosition().has_value())
      frc::SmartDashboard::PutNumber(
          m_name + " Absolute Position",
          m_controller.GetAbsoluteEncoderPosition().value());

    if (m_controller.GetAbsoluteEncoderPosition().has_value()) {
      Distance_t absEncValue = Distance_t(
          std::abs(m_controller.GetAbsoluteEncoderPosition().value()));

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
      ConsoleWriter.logInfo(m_name,
                            "Periodic: Movement with speed %f is not allowed",
                            m_latestSpeed);

      Stop();
    }
  }

  virtual void RunMotorVelocity(Velocity_t speed,
                                bool ignoreEncoder = false) = 0;

  void UseState(PidState setpoint) override {
    m_controller.RunToPosition(setpoint.position.value());
  }

  void RunMotorSpeedDefault(bool ignoreEncoder = false) override {
    RunMotorVelocity(m_config.defaultSpeed, ignoreEncoder);
  }

  void RunMotorPercentage(double percentSpeed,
                          bool ignoreEncoder = false) override {
    bool movementAllowed = IsMovementAllowed(percentSpeed, ignoreEncoder);
    frc::SmartDashboard::PutBoolean(m_name + " Movement Allowed",
                                    movementAllowed);

    if (!movementAllowed) {
      Stop();
      return;
    }

    DisablePid();

    frc::SmartDashboard::PutNumber(m_name + " Speed %", percentSpeed);

    m_controller.Set(percentSpeed);
  }

  Distance_t GetCurrentPosition() override {
    return Distance_t(m_controller.GetEncoderPosition());
  }

  void Stop() override {
    frc::SmartDashboard::PutNumber(m_name + " Speed %", 0);
    m_controller.Stop();
  }

  void ResetEncoder() override {
    ConsoleWriter.logInfo(m_name, "Reset encoder%s", "");
    m_controller.ResetEncoder();
  }

  bool AtHome() override {
    return AtLimitSwitchMin() || GetCurrentPosition() <= m_config.minDistance;
  }

  bool AtMax() override {
    return AtLimitSwitchMax() || GetCurrentPosition() >= m_config.maxDistance;
  }

  bool AtLimitSwitchMin() override {
    if (m_minLimitSwitch && m_minLimitSwitch.value()) {
      bool value = !m_minLimitSwitch.value()->Get();
      frc::SmartDashboard::PutBoolean(m_name + " Min Limit Switch", value);
      return value;
    }

    return false;
  }

  bool AtLimitSwitchMax() override {
    if (m_maxLimitSwitch && m_maxLimitSwitch.value()) {
      bool value = !m_maxLimitSwitch.value()->Get();
      frc::SmartDashboard::PutBoolean(m_name + " Max Limit Switch", value);
      return value;
    }

    return false;
  }

  frc2::CommandPtr MoveToPositionAbsolute(Distance_t position) override {
    return frc2::InstantCommand(
               [this, position] {
                 if (position < m_config.minDistance ||
                     position > m_config.maxDistance) {
                   ConsoleWriter.logWarning(
                       m_name,
                       "Attempting to move to position %f outside of boundary",
                       position.value());
                   return;
                 }

                 ConsoleWriter.logVerbose(m_name,
                                          "Moving to absolute position %f",
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

  frc2::CommandPtr ResetRelativeEncoder() {
    return frc2::InstantCommand([this] { ResetEncoder(); }).ToPtr();
  }

  frc::MechanismLigament2d *GetLigament() { return m_ligament2d; }

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
  ISingleAxisSubsystem<TDistance>::SingleAxisConfig m_config;
  std::string m_name;
  Distance_t m_goalPosition;
  bool m_pidEnabled;
  bool m_home;
  bool resetOccurred = false;
  double m_latestSpeed;
  frc2::CommandPtr m_resetEncCmd = frc2::InstantCommand([] {}).ToPtr();
  frc::MechanismLigament2d *m_ligament2d;
};
