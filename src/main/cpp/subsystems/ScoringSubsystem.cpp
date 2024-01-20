#include "subsystems/ScoringSubsystem.h"

ScoringSubsystem::ScoringSubsystem() {}

void ScoringSubsystem::Periodic() {}

void ScoringSubsystem::SimulationPeriodic() {}

void ScoringSubsystem::Stop() {
    m_vectorSpinnyBoi.Set(0);
    m_ampLowerSpinnyBoi.Set(0);
    m_ampUpperSpinnyBoi.Set(0);
    m_speakerLowerSpinnyBoi.Set(0);
    m_speakerUpperSpinnyBoi.Set(0);
}

void ScoringSubsystem::SpinVectorSide(ScoringDirection direction) {
    if (direction == ScoringDirection::AmpSide) {
        m_vectorSpinnyBoi.Set(ScoringConstants::kVectorSpeed);
        return;
    }

    m_vectorSpinnyBoi.Set(-ScoringConstants::kVectorSpeed);
}

void ScoringSubsystem::StartScoringRamp(ScoringDirection direction) {
    if (direction == ScoringDirection::AmpSide) {
        SpinAmp();
        return;
    }

    SpinSpeaker();
}

bool ScoringSubsystem::GetMotorAtScoringSpeed(ScoringDirection direction) {
    if (direction == ScoringDirection::AmpSide) {
        return CheckAmpSpeed();
    }

    return CheckSpeakerSpeed();
}

bool ScoringSubsystem::GetMotorFreeWheel(ScoringDirection direction) {
    if (direction == ScoringDirection::AmpSide) {
        return m_ampLowerSpinnyBoi.GetOutputCurrent() < ScoringConstants::kFreeSpinCurrentThreshold;
    }

    return m_speakerLowerSpinnyBoi.GetOutputCurrent() < ScoringConstants::kFreeSpinCurrentThreshold;
}

void ScoringSubsystem::SpinAmp() {
    m_ampLowerSpinnyBoi.Set(ScoringConstants::kAmpLowerSpeed);
    m_ampUpperSpinnyBoi.Set(ScoringConstants::kAmpUpperSpeed);
}

void ScoringSubsystem::SpinSpeaker() {
    m_speakerLowerSpinnyBoi.Set(ScoringConstants::kSpeakerLowerSpeed);
    m_speakerUpperSpinnyBoi.Set(ScoringConstants::kSpeakerUpperSpeed);
}

bool ScoringSubsystem::CheckAmpSpeed() {
    return m_ampEncoder.GetVelocity() >= MaxSpeedToRpm(ScoringConstants::kAmpLowerSpeed);
}

bool ScoringSubsystem::CheckSpeakerSpeed() {
    return m_speakerEncoder.GetVelocity() >= MaxSpeedToRpm(ScoringConstants::kSpeakerLowerSpeed);
}
