// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>

#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 2.4_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2 * std::numbers::pi};

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration
constexpr units::meter_t kTrackWidth =
    0.5588_m;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    0.5588_m;  // Distance between centers of front and back wheels on robot

// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;

// SPARK MAX CAN IDs
constexpr int kFrontRightDrivingCanId = 2;
constexpr int kRearRightDrivingCanId = 4;
constexpr int kFrontLeftDrivingCanId = 8;
constexpr int kRearLeftDrivingCanId = 6;

constexpr int kFrontRightTurningCanId = 1;
constexpr int kRearRightTurningCanId = 3;
constexpr int kFrontLeftTurningCanId = 7;
constexpr int kRearLeftTurningCanId = 5;

constexpr units::second_t kLoopTime = 0.022_s;
}  // namespace DriveConstants

namespace ModuleConstants {
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 13;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction =
    (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor =
    (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
    (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.04;
constexpr double kDrivingI = 0;
constexpr double kDrivingD = 0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 1;
constexpr double kTurningI = 0;
constexpr double kTurningD = 0;
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::CANSparkMax::IdleMode kDrivingMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;
constexpr rev::CANSparkMax::IdleMode kTurningMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 50_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;
constexpr double kDriveDeadband = 0.05;
}  // namespace OIConstants

constexpr uint8_t kLedAddress = 0x02;

// Motor IDs
namespace CANSparkMaxConstants {
constexpr int kIntakeSpinnyBoyID = 20;
constexpr int kWristRotationMotorID = 22;
}  // namespace CANSparkMaxConstants

namespace ArmConstants {
constexpr double kMaxRPM = 4000;
// Extension Constants
constexpr double kExtensionGearRatio = 36;
constexpr double kInPerRotation = -3.5 / kExtensionGearRatio;
constexpr double kMaxArmDistance = 12;
constexpr int kExtenderLimitSwitchPort = 2;
constexpr double kExtenderStepSize = 1;

constexpr double kExtenderSetP = 0.018386;
constexpr double kExtenderSetI = 0.0075;
constexpr double kExtenderSetD = 0.00062724;
constexpr double kExtenderSetIZone = 0.01;
constexpr double kExtenderSetFF = 0.000015;

// Arm Rotation Constants
constexpr int kRotationLimitSwitchHomePort = 0;
constexpr int kRotationLimitSwitchMaxPort = 1;
constexpr double kRotationHomeDegree = 60;
constexpr double kRotationMaxDegree = 145;
constexpr int kArmSoftLimitForwardDegrees = 65;
constexpr double kArmGearRatio = 197.14;
constexpr double kArmStepSize = 4;
constexpr double kAntiGravityPercentage = -0.05;

constexpr double kArmRotationSetP = 0.0018386;
constexpr double kArmRotationSetI = 0.0075;
constexpr double kArmRotationSetD = 0.00062724;
constexpr double kArmRotationSetIZone = 0.01;
constexpr double kArmRotationSetFF = 0.000015;

// Motor Constants
constexpr int kTicksPerMotorRotation = 42;
constexpr double kArmTicksPerDegree =
    (kTicksPerMotorRotation * kArmGearRatio) / 360.0;
constexpr double kForwardRotationsSoftLimit =
    (kArmSoftLimitForwardDegrees * kArmTicksPerDegree) / kTicksPerMotorRotation;
constexpr int kReverseRotationsSoftLimit = 0;

// Homing Speeds
constexpr double kRotationHomingSpeed = .15;
constexpr double kExtenderHomingSpeed = .66;
constexpr double kWristHomingSpeed = .33;

// Intake Constants
constexpr double kIntakeSpeed = 0.33;
constexpr double kOuttakeSpeed = 0.33;

// Wrist Constants
constexpr int kWristLimitSwitchPort = 0;
constexpr int kWristGearRatio = 125;
constexpr auto kWristDegreeLimit = 144;
constexpr double kWristStepSize = 4;

constexpr double kWristSetP = 0.0018386;
constexpr double kWristSetI = 0.0075;
constexpr double kWristSetD = 0.00062724;
constexpr double kWristSetIZone = 0.01;
constexpr double kWristSetFF = 0.000015;
}  // namespace ArmConstants