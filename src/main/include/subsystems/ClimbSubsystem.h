#pragma once

#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>

#include <memory>
#include <string>
#include <subzero/singleaxis/LinearSingleAxisSubsystem.cpp>

#include "Constants.h"
#include "utils/ConsoleLogger.h"

class ClimbSubsystem : public LinearSingleAxisSubsystem<SparkMaxController> {
 public:
  ClimbSubsystem(std::string name, SparkMaxController& controller,
                 ISingleAxisSubsystem<units::meter>::SingleAxisConfig config,
                 frc::MechanismObject2d* node = nullptr)
      : LinearSingleAxisSubsystem<SparkMaxController>(name, controller, config,
                                                      node) {}
  void Periodic() override {
    LinearSingleAxisSubsystem<SparkMaxController>::Periodic();
  }
};