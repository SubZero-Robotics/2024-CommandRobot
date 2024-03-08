#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <memory>

template <typename TDistance, typename TVelocity>
class ISingleAxisSubsystem : public frc2::SubsystemBase {
 public:
  // Need controller input
  virtual void RunMotorSpeed(TVelocity speed, bool ignoreEncoder);
  virtual void RunMotorSpeedDefault(bool invertDirection);
  virtual void RunMotorExternal(TVelocity speed, bool ignoreEncoder = false);
  virtual void UpdateMovement();
  virtual void ResetEncoder();
  virtual double GetCurrentPosition();
  virtual bool AtHome();
  virtual bool AtMax();
  virtual bool AtLimitSwitchHome();
  virtual bool AtLimitSwitchMax();
  virtual void MoveToPositionAbsolute(TDistance position);
  virtual void MoveToPositionRelative(TDistance position);
  virtual void Home();
  virtual bool GetIsMovingToPosition();
  virtual void StopMovement();
  virtual frc2::CommandPtr GetHomeCommand();
};