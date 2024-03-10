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
  using State = typename frc2::TrapezoidProfileSubsystem<TDistance>::State;

 public:
  // Need controller input
  virtual void RunMotorSpeed(TVelocity speed, bool ignoreEncoder) {}
  virtual void RunMotorSpeedDefault(bool invertDirection) {}
  virtual void RunMotorExternal(TVelocity speed, bool ignoreEncoder = false) {}
  virtual void UpdateMovement() {}
  virtual void ResetEncoder() {}
  virtual double GetCurrentPosition() { return 0; }
  virtual bool AtHome() { return false; }
  virtual bool AtMax() { return false; }
  virtual bool AtLimitSwitchHome() { return false; }
  virtual bool AtLimitSwitchMax() { return false; }
  virtual void MoveToPositionAbsolute(TDistance position) {}
  virtual void MoveToPositionRelative(TDistance position) {}
  virtual void Home() {}
  virtual bool GetIsMovingToPosition() { return false; }
  virtual void StopMovement() {}
  virtual frc2::CommandPtr GetHomeCommand() {
    return frc2::InstantCommand([] {}).ToPtr();
  }
  virtual void UseState(State setpoint) override{};
};