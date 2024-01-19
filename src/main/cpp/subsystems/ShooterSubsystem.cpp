#include <subsystems/ShooterSubsystem.h>

ShooterSubsystem::ShooterSubsystem(){}

void ShooterSubsystem::Periodic() {}

void ShooterSubsystem::SimulationPeriodic() {}

void ShooterSubsystem::Out() {
    m_SpinnyBoi1.Set(1);
    m_SpinnyBoi2.Set(-1
    );
}

void ShooterSubsystem::In() {

}

void ShooterSubsystem::Stop() { 
    m_SpinnyBoi1.Set(0);
    m_SpinnyBoi2.Set(0);
 }