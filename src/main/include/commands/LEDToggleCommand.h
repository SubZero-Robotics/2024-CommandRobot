// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "moduledrivers/ConnectorX.h"
#include "ColorConstants.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LEDToggle : public frc2::CommandHelper<frc2::Command, LEDToggle> {
   public:
    /**
     * Creates a new LEDYellow.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit LEDToggle(ConnectorX::ConnectorXBoard *leds)
        : m_leds{leds}, isFinished{false} {
    }

    void Execute() override {
        if (m_leds->compareColor(m_leds->getCurrentColor(ConnectorX::LedPort::P1), kPurpleColor)) {
            m_leds->setColor(ConnectorX::LedPort::P1, kYellowColor);
        } else {
            m_leds->setColor(ConnectorX::LedPort::P1, kPurpleColor);
        }
        m_leds->setPattern(ConnectorX::LedPort::P1, ConnectorX::PatternType::SetAll, true);

        isFinished = true;
    }

    bool IsFinished() override { return isFinished; }

   private:
    ConnectorX::ConnectorXBoard* m_leds;
    bool isFinished = false;
};