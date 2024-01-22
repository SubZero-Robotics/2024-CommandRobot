#include "subsystems/LedSubsystem.h"
#include "ColorConstants.h"

void LedSubsystem::Periodic() {

}

void LedSubsystem::SimulationPeriodic() {

}

frc2::DeferredCommand LedSubsystem::GetDeferredFromState(StateGetter stateGetter) {
    return frc2::DeferredCommand([this, stateGetter] { return ShowFromState(stateGetter); }, {this});
}

frc2::CommandPtr LedSubsystem::ShowFromState(StateGetter stateGetter) {
    auto state = stateGetter();

    switch (state) {
        case RobotState::Intaking:
            return Intaking();
        case RobotState::ScoringSpeaker:
            return ScoringSpeaker();
        case RobotState::ScoringAmp:
            return ScoringAmp();
        case RobotState::Stowing:
            return Stowing();
        case RobotState::Idling:
            return Idling();
        default:
            return frc2::InstantCommand([] {}).ToPtr();
    }
}

frc2::CommandPtr LedSubsystem::Intaking() {
    return setColorAndPattern(LedConstants::kIntakeLedPort, ColorConstants::kRed, PatternType::Blink);
}

frc2::CommandPtr LedSubsystem::ScoringSpeaker() {
    return setColorAndPattern(LedConstants::kSpeakerLedPort, ColorConstants::kGreen, PatternType::Blink);
}

frc2::CommandPtr LedSubsystem::ScoringAmp() {
    return setColorAndPattern(LedConstants::kAmpLedPort, ColorConstants::kBlue, PatternType::Blink);
}

frc2::CommandPtr LedSubsystem::Stowing() {
    return setColorAndPattern(LedConstants::kStowLedPort, ColorConstants::kPurple, PatternType::Breathe);
}

frc2::CommandPtr LedSubsystem::Idling() {
    return setColorAndPattern(LedConstants::kIdleLedPort, ColorConstants::kYellow, PatternType::SetAll, true);
}

frc2::CommandPtr LedSubsystem::setColorAndPattern(LedPort port, frc::Color8Bit color, PatternType pattern, bool oneShot,
    int16_t delay) {
    return frc2::InstantCommand([this, port] { m_connectorX.setLedPort(port); }, {this}).ToPtr()
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        frc2::InstantCommand([this, port, color] { m_connectorX.setColor(port, color); }, {this}).ToPtr()
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        frc2::InstantCommand([this, port, pattern, oneShot, delay]
            { m_connectorX.setPattern(port, pattern, oneShot, delay); }, {this}).ToPtr()
    );
}