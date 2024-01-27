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

typedef struct {
    DriveSubsystem *drive;
    ClimbSubsystem *leftClimb;
    ClimbSubsystem *rightClimb;
    IntakeSubsystem *intake;
    ScoringSubsystem *scoring;
    LedSubsystem *led;
} Subsystems_t;

class StateSubsystem : public frc2::SubsystemBase {
    public:
        StateSubsystem(Subsystems_t& subsystems);

        void IncrementState();

        frc2::CommandPtr UpdateState(RobotState newState);

        frc2::CommandPtr StartIntaking();

        frc2::CommandPtr StartScoringSpeaker();

        frc2::CommandPtr StartScoringAmp();
    
        frc2::CommandPtr StartScoringSubwoofer();

        frc2::CommandPtr StartLoading();

        frc2::CommandPtr StartManual();

        frc2::CommandPtr StartClimb();

        inline RobotState GetState() const {
            return m_currentState;
        };

    private:
        RobotState m_currentState;
        Subsystems_t &m_subsystems;
};