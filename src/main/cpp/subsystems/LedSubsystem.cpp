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
        case RobotState::Loaded:
            return Loaded();
        case RobotState::Manual:
            return Idling();
        default:
            return frc2::InstantCommand([] {}).ToPtr();
    }
}

frc2::CommandPtr LedSubsystem::Intaking() {
    return setZoneColorPattern(LedZone::Left, LedConstants::kIntakeLedPort, ColorConstants::kRed, PatternType::Chase,
        false, 60, false)
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Right, LedConstants::kIntakeLedPort, ColorConstants::kRed, PatternType::Chase,
            false, 60, true)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Front, LedConstants::kIntakeLedPort, ColorConstants::kRed, PatternType::SineRoll,
            false, 40, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Back, LedConstants::kIntakeLedPort, ColorConstants::kRed, PatternType::SineRoll,
            false, 40, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        syncAllZones()
    );
}

frc2::CommandPtr LedSubsystem::ScoringSpeaker() {
    return setZoneColorPattern(LedZone::Left, LedConstants::kSpeakerLedPort, ColorConstants::kYellow, PatternType::SetAll,
        true, 500, false)
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Right, LedConstants::kSpeakerLedPort, ColorConstants::kYellow, PatternType::SetAll,
            true, 500, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Front, LedConstants::kSpeakerLedPort, ColorConstants::kYellow, PatternType::SetAll,
            true, 500, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Back, LedConstants::kSpeakerLedPort, ColorConstants::kPurple, PatternType::Blink,
            false, 250, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        syncAllZones()
    );
}

frc2::CommandPtr LedSubsystem::ScoringAmp() {
    return setZoneColorPattern(LedZone::Left, LedConstants::kSpeakerLedPort, ColorConstants::kYellow, PatternType::SetAll,
        true, 500, false)
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Right, LedConstants::kSpeakerLedPort, ColorConstants::kYellow, PatternType::SetAll,
            true, 500, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Front, LedConstants::kSpeakerLedPort, ColorConstants::kPurple, PatternType::Blink,
            false, 250, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Back, LedConstants::kSpeakerLedPort, ColorConstants::kYellow, PatternType::SetAll,
            true, 500, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        syncAllZones()
    );
}

frc2::CommandPtr LedSubsystem::Loaded() {
    return setZoneColorPattern(LedZone::Left, LedConstants::kStowLedPort, ColorConstants::kGreen, PatternType::Blink,
        false, 750, false)
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Right, LedConstants::kStowLedPort, ColorConstants::kGreen, PatternType::Blink,
            false, 750, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Front, LedConstants::kStowLedPort, ColorConstants::kGreen, PatternType::Blink,
            false, 750, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Back, LedConstants::kStowLedPort, ColorConstants::kGreen, PatternType::Blink,
            false, 750, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        syncAllZones()
    );
}

frc2::CommandPtr LedSubsystem::Idling() {
    return setZoneColorPattern(LedZone::Left, LedConstants::kIdleLedPort, ColorConstants::kBlue, PatternType::Breathe,
        false, 10, false)
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Right, LedConstants::kIdleLedPort, ColorConstants::kBlue, PatternType::Breathe,
            false, 10, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Front, LedConstants::kIdleLedPort, ColorConstants::kBlue, PatternType::Breathe,
            false, 10, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Back, LedConstants::kIdleLedPort, ColorConstants::kBlue, PatternType::Breathe,
            false, 10, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        syncAllZones()
    );
}

frc2::CommandPtr LedSubsystem::Climbing() {
    return setZoneColorPattern(LedZone::Left, LedConstants::kIdleLedPort, ColorConstants::kBlue, PatternType::SineRoll,
        false, 10, false)
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Right, LedConstants::kIdleLedPort, ColorConstants::kBlue, PatternType::SineRoll,
            false, 10, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Front, LedConstants::kIdleLedPort, ColorConstants::kYellow, PatternType::SineRoll,
            false, 10, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Back, LedConstants::kIdleLedPort, ColorConstants::kYellow, PatternType::SineRoll,
            false, 10, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        syncAllZones()
    );
}

frc2::CommandPtr LedSubsystem::Error() {
    return setZoneColorPattern(LedZone::Left, LedConstants::kIntakeLedPort, ColorConstants::kRed, PatternType::Blink,
        false, 400, false)
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Right, LedConstants::kIntakeLedPort, ColorConstants::kRed, PatternType::Blink,
            false, 400, true)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Front, LedConstants::kIntakeLedPort, ColorConstants::kRed, PatternType::Blink,
            false, 400, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        setZoneColorPattern(LedZone::Back, LedConstants::kIntakeLedPort, ColorConstants::kRed, PatternType::Blink,
            false, 400, false)
    )
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        syncAllZones()
    );
}

frc2::CommandPtr LedSubsystem::setZoneColorPattern(LedZone zone, LedPort port, frc::Color8Bit color, PatternType pattern, bool oneShot,
    int16_t delay, bool reversed) {

    auto zoneIndex = static_cast<uint8_t>(zone);
    
    return frc2::InstantCommand([this, port, color, zoneIndex] { m_connectorX.setColor(port, color, zoneIndex); }, {this}).ToPtr()
    .AndThen(
        frc2::WaitCommand(0.02_s).ToPtr()
    )
    .AndThen(
        frc2::InstantCommand([this, port, pattern, oneShot, delay, zoneIndex, reversed]
            { m_connectorX.setPattern(port, pattern, oneShot, delay, zoneIndex, reversed); }, {this}).ToPtr()
    );
}

void LedSubsystem::createZones(LedPort port, std::vector<Commands::NewZone> &&zones) {
    m_connectorX.createZones(LedPort::P1, std::move(zones));
}

frc2::CommandPtr LedSubsystem::syncAllZones() {
    return frc2::InstantCommand([this] { m_connectorX.syncZones(LedConstants::kIntakeLedPort, 
            { (uint8_t)LedZone::Left, (uint8_t)LedZone::Right, (uint8_t)LedZone::Front,
            (uint8_t)LedZone::Back }); }).ToPtr();
}
