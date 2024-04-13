#pragma once

#include "subsystems/singleaxis/BaseSingleAxisSubsystem.h"

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder>
class RotationalSingleAxisSubsystem
    : public BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                                     TAbsoluteEncoder, units::degree> {
 public:
  RotationalSingleAxisSubsystem(
      std::string name,
      PidMotorController<TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder> &controller,
      ISingleAxisSubsystem<units::degree>::SingleAxisConfig config,
      units::meter_t armatureLength, frc::MechanismObject2d *node = nullptr)
      : BaseSingleAxisSubsystem<
            TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
            units::degree>{name, controller, config,
                           AutoConstants::kRotationalAxisConstraints, node},
        m_armatureLength{armatureLength} {}

  void Periodic() override {
    BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                            TAbsoluteEncoder, units::degree>::Periodic();

    if (BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                                TAbsoluteEncoder,
                                units::degree>::m_ligament2d) {
      BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                              TAbsoluteEncoder, units::degree>::m_ligament2d
          ->SetAngle(
              (BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                                       TAbsoluteEncoder,
                                       units::degree>::m_config.reversed
                   ? -BaseSingleAxisSubsystem<
                         TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder, units::degree>::GetCurrentPosition()
                   : BaseSingleAxisSubsystem<
                         TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder,
                         units::degree>::GetCurrentPosition()) +
              BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                                      TAbsoluteEncoder, units::degree>::m_config
                  .mechanismConfig.minimumAngle);
    }
  }

  void RunMotorVelocity(units::degrees_per_second_t speed,
                        bool ignoreEncoder = false) override {
    if (!BaseSingleAxisSubsystem<
            TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
            units::degree>::IsMovementAllowed(speed.value(), ignoreEncoder)) {
      return;
    }

    BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                            TAbsoluteEncoder, units::degree>::DisablePid();

    BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                            TAbsoluteEncoder, units::degree>::m_controller
        .RunWithVelocity(speed);
  }

 protected:
  units::meter_t m_armatureLength;
};