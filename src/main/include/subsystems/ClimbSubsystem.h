#pragma once

#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>
#include <subzero/logging/ConsoleLogger.h>
#include <subzero/singleaxis/LinearSingleAxisSubsystem.h>

#include <memory>
#include <string>

#include "Constants.h"

class ClimbSubsystem : public LinearSingleAxisSubsystem<SparkMaxController> {
 public:
  ClimbSubsystem(std::string name, SparkMaxController& controller,
                 ISingleAxisSubsystem<units::meter>::SingleAxisConfig config,
                 frc::MechanismObject2d* node = nullptr)
      : LinearSingleAxisSubsystem<SparkMaxController>(name, controller, config,
                                                      node) {}
};