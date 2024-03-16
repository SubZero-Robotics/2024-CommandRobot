#pragma once

#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>

#include <memory>

#include "Constants.h"
#include "subsystems/BaseSingleAxisSubsystem2.h"
#include "utils/ConsoleLogger.h"

class ClimbSubsystem
    : public LinearSingleAxisSubsystem<
          rev::CANSparkMax, rev::SparkPIDController, rev::SparkRelativeEncoder,
          rev::SparkAbsoluteEncoder> {
 public:
  ClimbSubsystem(std::string name,
                 PidMotorController<rev::CANSparkMax, rev::SparkPIDController,
                                    rev::SparkRelativeEncoder,
                                    rev::SparkAbsoluteEncoder> &controller,
                 ISingleAxisSubsystem2<units::meter>::SingleAxisConfig2 config)
      : LinearSingleAxisSubsystem<rev::CANSparkMax, rev::SparkPIDController,
                                  rev::SparkRelativeEncoder,
                                  rev::SparkAbsoluteEncoder>(name, controller,
                                                             config) {}
  void Periodic() override {
    LinearSingleAxisSubsystem<rev::CANSparkMax, rev::SparkPIDController,
                              rev::SparkRelativeEncoder,
                              rev::SparkAbsoluteEncoder>::Periodic();
  }
};