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
class ISingleAxisSubsystem2
    : public frc2::TrapezoidProfileSubsystem<TDistance> {
  using PidState = typename frc::TrapezoidProfile<TDistance>::State;

 public:
  // Need controller input
  virtual void RunMotorSpeed(TVelocity speed, bool ignoreEncoder = false) = 0;
  virtual void RunMotorSpeed(double percentSpeed,
                             bool ignoreEncoder = false) = 0;
  virtual void RunMotorSpeedDefault(bool invertDirection) = 0;
  virtual void ResetEncoder() = 0;
  virtual TDistance GetCurrentPosition() = 0;
  virtual bool AtHome() = 0;
  virtual bool AtMax() = 0;
  virtual bool AtLimitSwitchMin() = 0;
  virtual bool AtLimitSwitchMax() = 0;
  virtual frc2::CommandPtr MoveToPositionAbsolute(TDistance position) = 0;
  virtual frc2::CommandPtr MoveToPositionRelative(TDistance position) = 0;
  virtual frc2::CommandPtr Home() = 0;
  virtual bool IsEnabled() = 0;
  virtual void Disable() = 0;
  virtual void Enable() = 0;
  virtual void Stop() = 0;
};