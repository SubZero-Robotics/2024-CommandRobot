// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "moduledrivers/ConnectorX.h"
#include "subsystems/LedSubsystem.h"
#include "ColorConstants.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */

using namespace ConnectorX;

class LEDTogglePattern : public frc2::CommandHelper<frc2::Command, LEDTogglePattern> {
   public:
    /**
     * Creates a new LEDYellow.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit LEDTogglePattern(LedSubsystem *leds)
        : m_leds{leds}, isFinished{false} {
    }

    void Execute() override {
        switch(currentState) {
            case 0:
                m_leds->idlingLedPattern();
                break;
            case 1:
                m_leds->intakingLedPattern();
                break;
            case 2:
                m_leds->scoringAmpLedPattern();
                break;
            case 3:
                m_leds->scoringSpeakerLedPattern();
                break;
            case 4:
                m_leds->stowingLedPattern(); 
                break;
        }

        currentState++;
        currentState %= 5;
        isFinished = true;
    }

    bool IsFinished() override { return isFinished; }

   private:
    LedSubsystem* m_leds;
    uint8_t currentState = 0;
    bool isFinished = false;
};