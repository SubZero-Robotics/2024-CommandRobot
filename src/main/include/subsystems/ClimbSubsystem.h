#pragma once

#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>
#include <subsystems/BaseSingleAxisSubsystem.h>

#include <memory>

#include "Constants.h"
#include "utils/ConsoleLogger.h"

class ClimbSubsystem
    : public BaseSingleAxisSubsystem<rev::CANSparkMax,
                                     rev::SparkRelativeEncoder> {
 public:
  ClimbSubsystem(SingleAxisConfig &config, rev::CANSparkMax &motor,
                 rev::SparkRelativeEncoder &encoder, std::string name)
      : BaseSingleAxisSubsystem(config, motor, encoder, nullptr, nullptr, name,
                                true) {}

  void ResetEncoder() override { _enc.SetPosition(0); }

  double GetCurrentPosition() override {
    auto curPosition = _enc.GetPosition() * _config.distancePerRevolution;
    // ConsoleLogger::getInstance().logVerbose("Clinb Sbubby", "cur position
    // %f", curPosition);
    return curPosition;
  }

  void MoveRelative(double delta) {
    double newPosition = GetCurrentPosition() + delta;
    MoveToPosition(newPosition);
  }
};