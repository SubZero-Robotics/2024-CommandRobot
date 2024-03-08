#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <memory>

class ISingleAxisSubsystem : public frc2::SubsystemBase {
 public:
  // Need controller input
  virtual void RunMotorSpeed(double speed, bool ignoreEncoder) = 0;
  virtual void RunMotorSpeedDefault(bool invertDirection) = 0;
  virtual void RunMotorExternal(double speed, bool ignoreEncoder = false) = 0;
  virtual void UpdateMovement() = 0;
  virtual void ResetEncoder() = 0;
  virtual double GetCurrentPosition() = 0;
  virtual bool AtHome() = 0;
  virtual bool AtMax() = 0;
  virtual bool AtLimitSwitchHome() = 0;
  virtual bool AtLimitSwitchMax() = 0;
  virtual void Home() = 0;
  virtual void MoveToPosition(double) = 0;
  virtual bool GetIsMovingToPosition() = 0;
  virtual void StopMovement() = 0;
  virtual void JoystickMoveStep(double) = 0;
  virtual double IncrementTargetPosition(double) = 0;
  virtual frc2::CommandPtr GetHomeCommand() = 0;
};

template <typename TDistance, typename TVelocity>
class ISingleAxisSubsystem2 : public frc2::SubsystemBase {
 public:
  // Need controller input
  virtual void RunMotorSpeed(units::degrees_per_second speed, bool ignoreEncoder);
  virtual void RunMotorSpeedDefault(bool invertDirection);
  virtual void RunMotorExternal(units::degrees_per_second speed, bool ignoreEncoder = false);
  virtual void UpdateMovement();
  virtual void ResetEncoder();
  virtual double GetCurrentPosition();
  virtual bool AtHome();
  virtual bool AtMax();
  virtual bool AtLimitSwitchHome();
  virtual bool AtLimitSwitchMax();
  virtual void MoveToPositionAbsolute(units::degree position);
  virtual void MoveToPositionRelative(units::degree position);
  virtual void Home();
  virtual bool GetIsMovingToPosition();
  virtual void StopMovement();
  virtual frc2::CommandPtr GetHomeCommand();
};