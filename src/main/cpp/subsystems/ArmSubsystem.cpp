#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem()
    : BaseSingleAxisSubsystem(m_config, m_wristMotor, m_encoder, &min, nullptr,
                              "WRIST", "\033[92;40;4m") {
  _config = m_config;
  _controller = m_config.pid;
  _controller.SetTolerance(10, 10);
  m_encoder.SetPositionConversionFactor(1);
}

void ArmSubsystem::ResetEncoder() {}

double ArmSubsystem::GetCurrentPosition() {
  auto position = m_encoder.GetPosition() * _config.distancePerRevolution;

  if (position >= 350) position = 0;

  return position;
}