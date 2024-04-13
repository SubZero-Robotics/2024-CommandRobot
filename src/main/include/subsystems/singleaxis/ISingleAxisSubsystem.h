#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <memory>

#include "ColorConstants.h"

struct SingleAxisMechanism {
  units::meter_t minimumLength;
  units::degree_t minimumAngle;
  double lineWidth;
  frc::Color8Bit color;
};

template <typename Distance>
class ISingleAxisSubsystem {
 public:
  using Distance_t = units::unit_t<Distance>;
  using Velocity =
      units::compound_unit<Distance, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<Velocity>;
  using Acceleration =
      units::compound_unit<Velocity, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<Acceleration>;

  struct SingleAxisConfig {
    Distance_t minDistance;
    Distance_t maxDistance;
    Distance_t encoderDistancePerRevolution;
    std::optional<Distance_t> absoluteEncoderDistancePerRevolution;
    Velocity_t defaultSpeed;
    double velocityScalar;
    Distance_t tolerance;
    std::optional<frc::DigitalInput *> minLimitSwitch;
    std::optional<frc::DigitalInput *> maxLimitSwitch;
    bool reversed;
    SingleAxisMechanism mechanismConfig;

    SingleAxisConfig(
        Distance_t _minDistance, Distance_t _maxDistance,
        Distance_t _encoderDistancePerRevolution,
        std::optional<Distance_t> _absoluteEncoderDistancePerRevolution,
        Velocity_t _defaultSpeed, double _velocityScalar, Distance_t _tolerance,
        std::optional<frc::DigitalInput *> _minLimitSwitch,
        std::optional<frc::DigitalInput *> _maxLimitSwitch, bool _reversed,
        SingleAxisMechanism _mechanismConfig)
        : minDistance{_minDistance},
          maxDistance{_maxDistance},
          encoderDistancePerRevolution{_encoderDistancePerRevolution},
          absoluteEncoderDistancePerRevolution{
              _absoluteEncoderDistancePerRevolution},
          defaultSpeed{_defaultSpeed},
          velocityScalar{_velocityScalar},
          tolerance{_tolerance},
          minLimitSwitch{_minLimitSwitch},
          maxLimitSwitch{_maxLimitSwitch},
          reversed{_reversed},
          mechanismConfig{_mechanismConfig} {}
  };

  // Will disable position-based movements when called
  virtual void RunMotorVelocity(Velocity_t speed,
                                bool ignoreEncoder = false) = 0;
  // Will disable position-based movements when called
  virtual void RunMotorPercentage(double percentSpeed,
                                  bool ignoreEncoder = false) = 0;
  virtual void RunMotorSpeedDefault(bool ignoreEncoder = false) = 0;
  virtual void ResetEncoder() = 0;
  virtual Distance_t GetCurrentPosition() = 0;
  virtual bool AtHome() = 0;
  virtual bool AtMax() = 0;
  virtual bool AtLimitSwitchMin() = 0;
  virtual bool AtLimitSwitchMax() = 0;
  virtual frc2::CommandPtr MoveToPositionAbsolute(Distance_t position) = 0;
  virtual frc2::CommandPtr MoveToPositionRelative(Distance_t position) = 0;
  virtual frc2::CommandPtr Home() = 0;
  virtual bool IsEnabled() = 0;
  virtual void DisablePid() = 0;
  virtual void EnablePid() = 0;
  virtual void Stop() = 0;
};