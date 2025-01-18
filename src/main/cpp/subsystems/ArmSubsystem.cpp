#include "subsystems/ArmSubsystem.h"

void ArmSubsystem::SetP(double p) { m_PidController.SetI(p); }

void ArmSubsystem::SetI(double i) { m_PidController.SetI(i); }

void ArmSubsystem::SetD(double d) { m_PidController.SetI(d); }

void ArmSubsystem::SetFF(double ff) { m_PidController.SetFF(ff); }

double ArmSubsystem::GetP() { return m_PidController.GetP(); }
double ArmSubsystem::GetI() { return m_PidController.GetI(); }
double ArmSubsystem::GetD() { return m_PidController.GetD(); }
double ArmSubsystem::GetFF() { return m_PidController.GetFF(); }