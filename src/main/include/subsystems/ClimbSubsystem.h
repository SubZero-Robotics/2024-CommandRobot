#pragma once

#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>
#include <subzero/logging/ConsoleLogger.h>
#include <subzero/motor/IPidMotorController.h>
#include <subzero/singleaxis/LinearSingleAxisSubsystem.h>

#include <memory>
#include <string>

#include "Constants.h"

class ClimbSubsystem : public LinearSingleAxisSubsystem<IPidMotorController> {
 public:
  ClimbSubsystem(std::string name, IPidMotorController& controller,
                 ISingleAxisSubsystem<units::meter>::SingleAxisConfig config,
                 frc::MechanismObject2d* node = nullptr)
      : LinearSingleAxisSubsystem<IPidMotorController>(name, controller, config,
                                                       node) {}
};