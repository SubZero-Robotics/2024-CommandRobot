#ifndef I_SINGLE_AXIS_SUBSYSTEM_H
#define I_SINGLE_AXIS_SUBSYSTEM_H

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <memory>

class ISingleAxisSubsystem : public frc2::SubsystemBase {
public:
  // Need controller input
  virtual void RunMotorSpeed(double speed, bool ignoreEncoder) = 0;
  virtual void RunMotorSpeedDefault(bool invertDirection) = 0;
  virtual void RunMotorExternal(double speed) = 0;
  virtual void UpdateMovement() = 0;
  virtual void ResetEncoder() = 0;
  virtual double GetCurrentPosition() = 0;
  virtual bool AtHome() = 0;
  virtual bool AtMax() = 0;
  virtual bool AtLimitSwitchHome() = 0;
  virtual bool AtLimitSwitchMax() = 0;
  virtual void MoveToPosition(double position) = 0;
  virtual void Home() = 0;
  virtual bool GetIsMovingToPosition() = 0;
  virtual void StopMovement() = 0;
  virtual frc2::CommandPtr GetHomeCommand() = 0;
  virtual void JoystickMoveStep(double rotation) = 0;
  virtual double IncrementTargetPosition(double steps) = 0;
};

#endif