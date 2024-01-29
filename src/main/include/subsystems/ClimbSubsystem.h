#pragma once

#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>
#include <subsystems/BaseSingleAxisSubsystem.h>
#include <utils/ConsoleLogger.h>

#include <memory>

#include "Constants.h"

class ClimbSubsystem
    : public BaseSingleAxisSubsystem<rev::CANSparkMax,
                                     rev::SparkRelativeEncoder> {
 public:
  ClimbSubsystem(SingleAxisConfig &config, rev::CANSparkMax &motor,
                 rev::SparkRelativeEncoder &encoder,
                 frc::DigitalInput *minLimitSwitch, std::string name)
      : BaseSingleAxisSubsystem(config, motor, encoder, minLimitSwitch, nullptr,
                                name, true) {}

  void ResetEncoder() override { _enc.SetPosition(0); }

  double GetCurrentPosition() override { return _enc.GetPosition(); }

  void MoveRelative(double delta) {
    double newPosition = GetCurrentPosition() + delta;
    MoveToPosition(newPosition);
  }
};