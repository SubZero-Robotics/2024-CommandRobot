#pragma once

#include <functional>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <rev/CANSparkFlex.h>

#include "Constants.h"
#include "ColorConstants.h"

#include "Constants.h"
#include "commands/Funni.h"
#include "commands/IntakeInCommand.h"
#include "commands/IntakeOutCommand.h"
#include "commands/ScoreAmpCommand.h"
#include "commands/ScoreSpeakerCommand.h"
#include "commands/ScoreSubwooferCommand.h"
#include "commands/ExtendClimbCommand.h"
#include "commands/BalanceCommand.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LedSubsystem.h"
#include "utils/ShuffleboardLogger.h"

enum class RobotState {
    Manual = 0,
    ScoringSpeaker,
    ScoringAmp,
    ScoringSubwoofer,
    Loaded,
    Intaking,
    Climb
};

typedef struct {
    DriveSubsystem *drive;
    ClimbSubsystem *leftClimb;
    ClimbSubsystem *rightClimb;
    IntakeSubsystem *intake;
    ScoringSubsystem *scoring;
    LedSubsystem *led;
} Subsystems_t;

typedef std::function<RobotState ()> StateGetter;

class StateSubsystem : public frc2::SubsystemBase {
    public:
        StateSubsystem(Subsystems_t& subsystems);

        void incrementState();
        frc2::CommandPtr updateState(RobotState newState);

        frc2::CommandPtr startIntaking();

        frc2::CommandPtr startScoringSpeaker();

        frc2::CommandPtr startScoringAmp();
    
        frc2::CommandPtr startScoringSubwoofer();

        frc2::CommandPtr startLoading();

        frc2::CommandPtr startManual();

        frc2::CommandPtr startClimb();

        inline RobotState getState() const {
            return m_currentState;
        };

    private:
        RobotState m_currentState;
        Subsystems_t &m_subsystems;
};