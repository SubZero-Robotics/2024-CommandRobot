#pragma once

#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>

#include <memory>
#include <string>

#include "Constants.h"
#include "subsystems/singleaxis/LinearSingleAxisSubsystem.h"
#include "utils/ConsoleLogger.h"

class ClimbSubsystem
    : public LinearSingleAxisSubsystem<
          rev::CANSparkMax, rev::SparkPIDController, rev::SparkRelativeEncoder,
          rev::SparkAbsoluteEncoder> {
 public:
  ClimbSubsystem(std::string name,
                 PidMotorController<rev::CANSparkMax, rev::SparkPIDController,
                                    rev::SparkRelativeEncoder,
                                    rev::SparkAbsoluteEncoder>& controller,
                 ISingleAxisSubsystem<units::meter>::SingleAxisConfig config,
                 frc::MechanismObject2d* node = nullptr)
      : LinearSingleAxisSubsystem<rev::CANSparkMax, rev::SparkPIDController,
                                  rev::SparkRelativeEncoder,
                                  rev::SparkAbsoluteEncoder>(name, controller,
                                                             config, node) {}
  void Periodic() override {
    LinearSingleAxisSubsystem<rev::CANSparkMax, rev::SparkPIDController,
                              rev::SparkRelativeEncoder,
                              rev::SparkAbsoluteEncoder>::Periodic();
  }
};