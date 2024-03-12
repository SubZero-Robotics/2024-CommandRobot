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

#include "Constants.h"
#include "subsystems/ISingleAxisSubsystem.h"
#include "utils/ConsoleLogger.h"
#include "utils/PidMotorController.h"
#include "utils/ShuffleboardLogger.h"

template <typename TController, typename TEncoder, typename TDistance>
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

  BaseSingleAxisSubsystem2(
      std::string name, PidMotorController<TController, TEncoder> &controller,
      ISingleAxisSubsystem2<TDistance>::SingleAxisConfig2 config)
      : frc2::TrapezoidProfileSubsystem<
            TDistance>{AutoConstants::kSingleAxisConstraints},
        m_minLimitSwitch{config.minLimitSwitch},
        m_maxLimitSwitch{config.maxLimitSwitch},
        m_controller{controller},
        m_config{config},
        m_name{name},
        m_enabled{false} {}

  void Periodic() override {
    frc2::TrapezoidProfileSubsystem<TDistance>::Periodic();
  }

  void UseState(PidState setpoint) override {
    m_controller.RunToPosition(setpoint.position /
                               m_config.distancePerRevolution);
  }

  void RunMotorSpeedDefault() override {}

  void RunMotorSpeed(double percentSpeed, bool ignoreEncoder = false) override {
    m_controller.RunWithVelocity(percentSpeed);
  }

  void Stop() override { RunMotorSpeed(0); }

  void ResetEncoder() override { m_controller.ResetEncoders(); }

  virtual Distance_t GetCurrentPosition() override {
    return m_controller.GetEncoderPosition() * m_config.distancePerRevolution;
  }

  // TODO
  bool AtHome() override { return false; }

  bool AtMax() override { return false; }

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
    m_goalPosition = position;
    return frc2::cmd::RunOnce(
        [this, position] {
          frc2::TrapezoidProfileSubsystem<TDistance>::SetGoal(position);
        },
        {this});
  }

  frc2::CommandPtr MoveToPositionRelative(Distance_t position) override {
    return MoveToPositionAbsolute(m_goalPosition + position);
  }

  frc2::CommandPtr Home() override {
    return frc2::FunctionalCommand(
               // onInit
               [this] { Stop(); },
               // onExecute
               [this] { RunMotorSpeedDefault(); },
               // onEnd
               [this](bool interrupted) {
                 Stop();
                 ResetEncoder();
               },
               // isFinished
               [this] { return AtLimitSwitchMin(); }, {this})
        .ToPtr();
  }

  bool IsEnabled() override { return m_enabled; }

  // TODO: Come back to this
  void Disable() override {
    m_enabled = false;
    frc2::TrapezoidProfileSubsystem<TDistance>::Disable();
  }

  void Enable() override {
    m_enabled = true;
    frc2::TrapezoidProfileSubsystem<TDistance>::Enable();
  }

 protected:
  std::optional<frc::DigitalInput *> m_minLimitSwitch;
  std::optional<frc::DigitalInput *> m_maxLimitSwitch;
  PidMotorController<TController, TEncoder> &m_controller;
  ISingleAxisSubsystem2<TDistance>::SingleAxisConfig2 m_config;
  std::string m_name;
  Distance_t m_goalPosition;
  bool m_enabled;
  bool m_home;
};

template <typename TController, typename TEncoder>
class RotationalSingleAxisSubsystem
    : public BaseSingleAxisSubsystem2<TController, TEncoder, units::degree> {
 public:
  RotationalSingleAxisSubsystem(
      std::string name, PidMotorController<TController, TEncoder> &controller,
      ISingleAxisSubsystem2<units::degree>::SingleAxisConfig2 config)
      : BaseSingleAxisSubsystem2<TController, TEncoder, units::degree>{
            name, controller, config} {}
};

template <typename TController, typename TEncoder>
class LinearSingleAxisSubsystem
    : public BaseSingleAxisSubsystem2<TController, TEncoder, units::meter> {
 public:
  LinearSingleAxisSubsystem(
      std::string name, PidMotorController<TController, TEncoder> &controller,
      ISingleAxisSubsystem2<units::degree>::SingleAxisConfig2 config)
      : BaseSingleAxisSubsystem2<TController, TEncoder, units::meter>{
            name, controller, config} {}
};

template <typename Motor, typename Encoder>
class BaseSingleAxisSubsystem : public ISingleAxisSubsystem {
 public:
  enum ConfigConstants {
    MOTOR_DIRECTION_NORMAL = 1,
    MOTOR_DIRECTION_REVERSED = -1,
    UNUSED_DIO_PORT = -1
  };

  enum class AxisType { Rotational, Linear };

  /**
   * @brief Configuration for a single axis of absolute movement
   *
   * @param type Rotational or Linear direction
   * @param pid PIDController for moving the axis along a profile
   * @param minDistance Minimum distance in your choice of linear or
   * rotational units
   * @param maxDistance Maximum distance in your choice of linear or
   * rotational units
   * @param distancePerRevolution Distance the axis moves per revolution of
   * the motor
   * @param motorMultiplier Set to 1.0 or -1.0 to reverse motor direction.
   * Negative is always decreasing distance
   * @param pidResultMultiplier Multiply the PID.Calculcate result by this
   * value
   */
  struct SingleAxisConfig {
    AxisType type;
    frc::PIDController pid;
    // TODO: Make a velocity PID
    double minDistance;
    double maxDistance;
    double distancePerRevolution;
    double stepSize;
    double motorMultiplier = 1.0;
    double pidResultMultiplier = 1.0;
    int minLimitSwitchPort = UNUSED_DIO_PORT;
    int maxLimitSwitchPort = UNUSED_DIO_PORT;
    double defaultMovementSpeed = 0.2;
  };

  BaseSingleAxisSubsystem(SingleAxisConfig &cfg, Motor &motor, Encoder &encoder,
                          frc::DigitalInput *minSwitch,
                          frc::DigitalInput *maxSwitch, std::string prefix,
                          bool log = false)
      : _motor(motor),
        _enc(encoder),
        _config(cfg),
        _controller(cfg.pid),
        _isHoming(false),
        _isMovingToPosition(false),
        _targetPosition(0),
        _prefix(prefix),
        _log(log),
        _minLimitSwitch(minSwitch),
        _maxLimitSwitch(maxSwitch) {
    _config.defaultMovementSpeed =
        std::clamp(_config.defaultMovementSpeed, -1.0, 1.0);
  }

  Motor *getMotor() { return _motor; }

  /**
   * @brief Run motor at the specified speed.
   * ! Make sure to use _config.motorMultiplier to invert if needed!
   * This method will also prevent movement in certain directions if at a
   * limit. Use RunMotorExternal if moving the motor from an external command
   *
   * @param speed Percentage speed
   */
  void RunMotorSpeed(double speed, bool ignoreEncoder = false) override {
    if (_log)
      ConsoleLogger::getInstance().logVerbose(_prefix, "MUL IS %.2f",
                                              _config.motorMultiplier);
    speed *= _config.motorMultiplier;
    speed = std::clamp(speed, -1.0, 1.0);
    if (_log)
      ConsoleLogger::getInstance().logVerbose(_prefix, "SPEED IS %.2f", speed);

    bool homeState = ignoreEncoder ? AtLimitSwitchHome() : AtHome();
    if (homeState) {
      if (_log) ConsoleLogger::getInstance().logInfo(_prefix, "AT HOME %s", "");
      if (speed < 0) {
        if (_log)
          ConsoleLogger::getInstance().logVerbose(
              _prefix, "SETTING SPEED TO: %.2f", speed);
        _motor.Set(speed);
        return;
      }

      if (_log)
        ConsoleLogger::getInstance().logVerbose(
            _prefix, "NOT MOVING; AT HOME; speed=%.2f", speed);

      _motor.Set(0);
      return;
    }

    else if (AtMax()) {
      if (_log) ConsoleLogger::getInstance().logInfo(_prefix, "AT MAX %s", "");
      if (speed > 0) {
        if (_log)
          ConsoleLogger::getInstance().logVerbose(
              _prefix, "SETTING SPEED TO: %.2f", speed);
        _motor.Set(speed);
        return;
      }

      if (_log)
        ConsoleLogger::getInstance().logVerbose(
            _prefix, "NOT MOVING; AT MAX; speed=%.2f", speed);
      _motor.Set(0);
      return;
    } else {
      if (_log)
        ConsoleLogger::getInstance().logVerbose(
            _prefix, "SETTING SPEED TO: %.2f", speed);
      _motor.Set(speed);
    }
  }

  void RunMotorSpeedDefault(bool invertDirection = false) override {
    RunMotorSpeed(invertDirection ? -_config.defaultMovementSpeed
                                  : _config.defaultMovementSpeed);
  }

  /**
   * @brief Call this one from a joystick-bound command to override current
   * movement
   *
   * @param speed Percentage speed
   */
  void RunMotorExternal(double speed, bool ignoreEncoder = false) override {
    // TODO: constant
    if (abs(speed) <= 0.05) {
      if (_isMovingToPosition)
        return;  // Don't set the motor and overwrite a potential
                 // automated movement

      // TODO: Use velocity PID instead
      if (_config.type == AxisType::Rotational)
        RunMotorSpeed(ArmConstants::kAntiGravityPercentage);  // Make 'er hover!
      else
        _motor.Set(0);

      return;
    }

    // Overwrite current automated position with joystick input
    if (_isMovingToPosition) {
      StopMovement();
    }

    RunMotorSpeed(speed, ignoreEncoder);
  }

  void JoystickMoveStep(double rotation) override {
    auto steps = rotation * _config.stepSize;
    auto newTarget = IncrementTargetPosition(steps);
    MoveToPosition(newTarget);
  }

  double IncrementTargetPosition(double steps) override {
    return std::clamp(_targetPosition + steps, _config.minDistance,
                      _config.maxDistance);
  }

  void UpdateMovement() override {
    if (_isMovingToPosition) {
      if (_log)
        ConsoleLogger::getInstance().logInfo(
            _prefix, "Target position: %.2f %s", _targetPosition,
            _config.type == AxisType::Linear ? "in" : "deg");

      auto res = _controller.Calculate(GetCurrentPosition(), _targetPosition) *
                 _config.pidResultMultiplier;
      auto clampedRes = std::clamp(res, -1.0, 1.0);
      if (_log)
        ConsoleLogger::getInstance().logInfo(_prefix, "Clamped Res: %.2f",
                                             clampedRes);
      ShuffleboardLogger::getInstance().logInfo(_prefix + " TargetPos",
                                                _targetPosition);

      if (_controller.AtSetpoint()) {
        ConsoleLogger::getInstance().logInfo(_prefix, "REACHED GOAL %s", "");
        StopMovement();
        return;
      }

      ShuffleboardLogger::getInstance().logVerbose(
          _prefix, "amperage %f", _motor.GetOutputCurrent(), "");
      ConsoleLogger::getInstance().logVerbose(_prefix, "amperage %f",
                                              _motor.GetOutputCurrent(), "");

      RunMotorSpeed(clampedRes);
    }
  }

  virtual void ResetEncoder() = 0;

  virtual double GetCurrentPosition() = 0;

  bool AtHome() override {
    if (_minLimitSwitch) {
      if (AtLimitSwitchHome()) {
        ResetEncoder();
        if (_log)
          ConsoleLogger::getInstance().logInfo(_prefix, "AT HOME SWITCH %s",
                                               "");
        return true;
      }
    }

    // TODO: Constant wrap-around value
    if (GetCurrentPosition() <= _config.minDistance ||
        GetCurrentPosition() >= 350.0) {
      if (_log)
        ConsoleLogger::getInstance().logInfo(_prefix, "AT HOME ENCODER %s", "");
      return true;
    }

    return false;
  }

  inline bool AtMax() override {
    if (_maxLimitSwitch) {
      if (AtLimitSwitchMax()) {
        if (_log)
          ConsoleLogger::getInstance().logInfo(_prefix, "AT MAX SWITCH %s", "");
        return true;
      }
    }

    // TODO: Constant wrap-around value
    if (GetCurrentPosition() >= _config.maxDistance &&
        GetCurrentPosition() < 350.0) {
      if (_log)
        ConsoleLogger::getInstance().logInfo(_prefix, "AT MAX ENCODER %s", "");

      return true;
    }

    return false;
  }

  inline bool AtLimitSwitchHome() override {
    if (_minLimitSwitch) {
      auto state = !_minLimitSwitch->Get();
      if (_log)
        ConsoleLogger::getInstance().logVerbose(_prefix, "MIN LIMIT SWITCH %d",
                                                state);
      return state;
    }

    return false;
  }

  inline bool AtLimitSwitchMax() override {
    if (_maxLimitSwitch) {
      auto state = !_maxLimitSwitch->Get();
      if (_log)
        ConsoleLogger::getInstance().logVerbose(_prefix, "MAX LIMIT SWITCH %d",
                                                state);
      return state;
    }

    return false;
  }

  void MoveToPosition(double position) override {
    if (_log)
      ConsoleLogger::getInstance().logInfo(_prefix, "Moving to %.2f", position);
    _isMovingToPosition = true;
    _targetPosition =
        std::clamp(position, _config.minDistance, _config.maxDistance);
  }

  void Home() override {
    if (_log)
      ConsoleLogger::getInstance().logInfo(_prefix, "Set homing to true %s",
                                           "");
    StopMovement();
    _isHoming = true;
  }

  inline bool GetIsMovingToPosition() override { return _isMovingToPosition; }

  inline void StopMovement() override {
    if (_log)
      ConsoleLogger::getInstance().logInfo(_prefix, "Movement stopped %s", "");
    _isHoming = false;
    _isMovingToPosition = false;
    _motor.Set(0);
  }

  frc2::CommandPtr GetHomeCommand() override {
    return frc2::FunctionalCommand(
               [this] { Home(); },
               // Ignore the home encoder value since it starts at 0
               [this] { RunMotorSpeed(_config.defaultMovementSpeed, true); },
               [this](bool interrupted) {
                 StopMovement();
                 ResetEncoder();
               },
               // Finish once limit switch is hit
               [this] { return AtLimitSwitchHome(); }, {this})
        .ToPtr();
  }

  void Periodic() override { UpdateMovement(); }

  static inline bool IsValidPort(int port) { return port >= 0 && port < 10; }

 protected:
  Motor &_motor;
  Encoder &_enc;
  SingleAxisConfig &_config;
  frc::PIDController _controller;
  bool _isHoming = false;
  bool _isMovingToPosition = false;
  double _targetPosition = 0.0;
  std::string _prefix;
  bool _log;

 private:
  frc::DigitalInput *_minLimitSwitch = nullptr;
  frc::DigitalInput *_maxLimitSwitch = nullptr;
};