#include "subsystems/ArmSubsystem.h"

void ArmSubsystem::SetP(double p) { m_PidController.SetI(p); }

void ArmSubsystem::SetI(double i) { m_PidController.SetI(i); }

void ArmSubsystem::SetD(double d) { m_PidController.SetD(d); }

void ArmSubsystem::SetFF(double ff) { m_PidController.SetFF(ff); }

double ArmSubsystem::GetP() { return m_PidController.GetP(); }
double ArmSubsystem::GetI() { return m_PidController.GetI(); }
double ArmSubsystem::GetD() { return m_PidController.GetD(); }
double ArmSubsystem::GetFF() { return m_PidController.GetFF(); }

void ArmSubsystem::MoveArmAbsolute(units::degree_t position) {
  // if (position < m_config.minDistance || position > m_config.maxDistance) {
  //   ConsoleWriter.logWarning(
  //       m_name, "Attempting to move to position %f outside of boundary.",
  //       position.value());
  //   return;
  // }

  // ConsoleWriter.logVerbose(m_name, "Moving to absolute position %f",
  //                          position.value());

  // m_goalPosition = position;
  // EnablePid();
  // frc2::TrapezoidProfileSubsystem<units::degree>::SetGoal(position);
}