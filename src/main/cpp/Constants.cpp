// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"

namespace AutoConstants {

const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};

const frc::TrapezoidProfile<units::degree>::Constraints
    kRotationalAxisConstraints{360_deg_per_s * 2.2, 360_deg_per_s_sq * 2.2};

const frc::TrapezoidProfile<units::meter>::Constraints kLinearAxisConstraints{
    1_fps * 10, 0.75_fps_sq * 20};

}  // namespace AutoConstants
