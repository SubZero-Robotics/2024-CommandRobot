#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/interfaces/Gyro.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <string>

class IGyroContainer {
 public:
  IGyroContainer(std::string name) : m_name(name) {}
  virtual frc::Rotation2d GetRotation() = 0;
  virtual void Reset() = 0;
  virtual bool HasError() = 0;
  virtual std::optional<std::string> GetStatusMessage() = 0;

  std::string GetName() const { return m_name; }

 protected:
  std::string m_name;
};

template <typename T>
class GyroContainer : public IGyroContainer {
 public:
  GyroContainer(T& gyro, std::string name)
      : IGyroContainer(name), m_gyro(gyro) {}

  frc::Rotation2d GetRotation() override { return m_gyro.GetRotation2d(); }

  void Reset() override { m_gyro.Reset(); }

 protected:
  T& m_gyro;
};

class PigeonGyroContainer
    : public GyroContainer<ctre::phoenix6::hardware::Pigeon2> {
 public:
  /// @brief
  /// @param gyro The pigeon gyro being passed in
  PigeonGyroContainer(ctre::phoenix6::hardware::Pigeon2& gyro, std::string name)
      : GyroContainer<ctre::phoenix6::hardware::Pigeon2>(gyro, name) {}

  bool HasError() override {
    return m_gyro.GetFaultField().GetStatus() != ctre::phoenix::StatusCode::OK;
  }

  std::optional<std::string> GetStatusMessage() override {
    return m_gyro.GetFaultField().GetStatus().GetName();
  }
};

class Adxrs450Gyro : public GyroContainer<frc::ADXRS450_Gyro> {
 public:
  /// @brief
  /// @param gyro The ADXRS450 Gyro gyro being passed in
  Adxrs450Gyro(frc::ADXRS450_Gyro& gyro, std::string name)
      : GyroContainer<frc::ADXRS450_Gyro>(gyro, name) {}

  bool HasError() override { return !m_gyro.IsConnected(); }

  std::optional<std::string> GetStatusMessage() override {
    return HasError() ? "Not Connected" : "Connected";
  }
};