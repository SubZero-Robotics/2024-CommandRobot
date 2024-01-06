#ifndef BASE_SINGLE_AXIS_SUBSYSTEM_H
#define BASE_SINGLE_AXIS_SUBSYSTEM_H

#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/FunctionalCommand.h>
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
#include "utils/ShuffleboardLogger.h"

// ! TODO: Update comment to match!
/**
 * @brief A base class for a single-axis subsystem
 *
 * @example BaseSingleAxisSubsystem<rev::CANSparkMax,
 units::meters>::SingleAxisConfig config = {
 BaseSingleAxisSubsystem<rev::CANSparkMax, units::meters>::AxisType::Linear, //
 type frc::ProfilePIDController(1.3, 0.0, 0.7,
            frc::TrapezoidProfile<units::meters>::Constraints(1.75_mps,
 0.75_mps_sq), 20_ms   // kDt (s)
        ),   // PID
        0,      // min distance
        200,    // max distance
        30,     // distance per revolution in linear units
        1,      // motor direction
        0,      // min limit switch port
        1,      // max limit switch port
        2,      // encoder port
        0.33    // default movement speed of 33%
    };

    rev::CANSparkMax m_leadRotationMotor{
        CANSparkMaxConstants::kLeadRotationMotorID,
        rev::CANSparkMax::MotorType::kBrushless};

    BaseSingleAxisSubsystem<rev::CANSparkMax, units::meters> singleAxis =
 BaseSingleAxisSubsystem<rev::CANSparkMax, units::meters>(config,
 m_leadRotationMotor);
 *
 * @tparam Motor Any motor that supports Set(percent)
 * @tparam Unit Position unit (units::meters, etc.)
 */
template <typename Motor, typename Encoder, typename Unit, typename Unit_t>
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
     */
    struct SingleAxisConfig {
        AxisType type;
        frc::PIDController pid;
        double minDistance;
        double maxDistance;
        double distancePerRevolution;
        double stepSize;
        double motorMultiplier = 1.0;
        int minLimitSwitchPort = UNUSED_DIO_PORT;
        int maxLimitSwitchPort = UNUSED_DIO_PORT;
        double defaultMovementSpeed = 0.2;
    };

    BaseSingleAxisSubsystem(SingleAxisConfig &cfg, Motor &motor,
                            Encoder &encoder, frc::DigitalInput *minSwitch,
                            frc::DigitalInput *maxSwitch, std::string prefix,
                            std::string ansiPrefixModifiers = "",
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
            consoleLogger.logVerbose(_prefix, "MUL IS %.2f", _config.motorMultiplier);
        speed *= _config.motorMultiplier;
        speed = std::clamp(speed, -1.0, 1.0);
        if (_log)
            consoleLogger.logVerbose(_prefix, "SPEED IS %.2f", speed);

        bool homeState = ignoreEncoder ? AtLimitSwitchHome() : AtHome();
        if (homeState) {
            if (_log)
                consoleLogger.logInfo(_prefix, "AT HOME");
            if (speed < 0) {
                if (_log)
                    consoleLogger.logVerbose(_prefix, "SETTING SPEED TO: %.2f", speed);
                _motor.Set(speed);
                return;
            }

            if (_log)
                consoleLogger.logVerbose(_prefix, "NOT MOVING; AT HOME; speed=%.2f", speed);

            _motor.Set(0);
            return;
        }

        else if (AtMax()) {
            if (_log)
                consoleLogger.logInfo(_prefix, "AT MAX");
            if (speed > 0) {
                if (_log)
                    consoleLogger.logVerbose(_prefix, "SETTING SPEED TO: %.2f", speed);
                _motor.Set(speed);
                return;
            }

            if (_log)
                consoleLogger.logVerbose(_prefix, "NOT MOVING; AT MAX; speed=%.2f", speed);
            _motor.Set(0);
            return;
        } else {
            if (_log)
                consoleLogger.logVerbose(_prefix, "SETTING SPEED TO: %.2f", speed);
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
    void RunMotorExternal(double speed) override {
        // TODO: constant
        if (abs(speed) <= 0.05) {
            if (_isMovingToPosition)
                return;  // Don't set the motor and overwrite a potential
                         // automated movement

            if (_config.type == AxisType::Rotational)
                RunMotorSpeed(
                    ArmConstants::kAntiGravityPercentage);  // Make 'er hover!
            else
                _motor.Set(0);

            return;
        }

        // Overwrite current automated position with joystick input
        if (_isMovingToPosition) {
            StopMovement();
        }

        RunMotorSpeed(speed);
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
                consoleLogger.logInfo(_prefix, "Target position: %.2f %s",
                    Unit_t(_targetPosition).value(), _config.type == AxisType::Linear ? "in" : "deg");

            auto res = _controller.Calculate(GetCurrentPosition(), _targetPosition);
            auto clampedRes = std::clamp(res, -1.0, 1.0) * 0.65;

            if (_log)
                consoleLogger.logInfo(_prefix, "Clamped Res: %.2f", clampedRes);
            
            shuffleboardLogger.logInfo(_prefix + " TargetPos", _targetPosition);

            if (_controller.AtSetpoint()) {
                consoleLogger.logInfo(_prefix, "REACHED GOAL");
                StopMovement();
                return;
            }

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
                    consoleLogger.logInfo(_prefix, "AT HOME SWITCH");

                return true;
            }
        }

        // TODO: Constant wrap-around value
        if (GetCurrentPosition() <= _config.minDistance ||
            GetCurrentPosition() >= 350.0) {
            if (_log)
                consoleLogger.logInfo(_prefix, "AT HOME ENCODER");

            return true;
        }

        return false;
    }

    inline bool AtMax() override {
        if (_maxLimitSwitch) {
            if (AtLimitSwitchMax()) {
                if (_log)
                consoleLogger.logInfo(_prefix, "AT MAX SWITCH");

                return true;
            }
        }

        if (GetCurrentPosition() >= _config.maxDistance &&
            GetCurrentPosition() < 350) {
            if (_log)
                consoleLogger.logInfo(_prefix, "AT MAX ENCODER");

            return true;
        }

        return false;
    }

    inline bool AtLimitSwitchHome() override {
        if (_minLimitSwitch) {
            auto state = !_minLimitSwitch->Get();
            if (_log)
                consoleLogger.logVerbose(_prefix, "MIN LIMIT SWITCH %d", state);

            return state;
        }

        return false;
    }

    inline bool AtLimitSwitchMax() override {
        if (_maxLimitSwitch) {
            auto state = !_maxLimitSwitch->Get();
            if (_log)
                consoleLogger.logVerbose(_prefix, "MAX LIMIT SWITCH %d", state);

            return state;
        }

        return false;
    }

    void MoveToPosition(double position) override {
        if (_log)
            consoleLogger.logInfo(_prefix, "Moving to %.2f", position);

        _isMovingToPosition = true;
        _targetPosition = position;
    }

    void Home() override {
        if (_log)
            consoleLogger.logInfo(_prefix, "Set homing to true");

        StopMovement();
        _isHoming = true;
    }

    inline bool GetIsMovingToPosition() override { return _isMovingToPosition; }

    inline void StopMovement() override {
        if (_log)
            consoleLogger.logInfo(_prefix, "Movement stopped");

        _isHoming = false;
        _isMovingToPosition = false;
        _motor.Set(0);
    }

    frc2::CommandPtr GetHomeCommand() override {
        return frc2::FunctionalCommand(
                   [this] { Home(); },
                   // Ignore the home encoder value since it starts at 0
                   [this] {
                       RunMotorSpeed(_config.defaultMovementSpeed, true);
                   },
                   [this](bool interrupted) {
                       StopMovement();
                       ResetEncoder();
                   },
                   // Finish once limit switch is hit
                   [this] { return AtLimitSwitchHome(); }, {this})
            .ToPtr();
    }

    void Periodic() override {
        // auto res = frc::SmartDashboard::GetNumber(_prefix + " Position Set",
        // 0); if (_log)
        //     Logging::logToStdOut(_prefix, "Shuffleboard Position" +
        //     std::to_string(res),
        //                          Logging::Level::INFO);
        // MoveToPosition(res);
        UpdateMovement();
    }

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

#endif