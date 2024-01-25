#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Constants.h"
#include "subsystems/ClimbSubsystem.h"

class ExtendClimbCommand : public frc2::CommandHelper<frc2::Command, ExtendClimbCommand> {
    public:
        ExtendClimbCommand(ClimbSubsystem *climber, std::function<double()> inExtendFunc,
                      std::function<double()> outExtendFunc) 
        : m_climbSubsystem(climber), m_inExtend(inExtendFunc),
        m_outExtend(outExtendFunc) {
            AddRequirements(climber);
        }

        void Execute() override {
            auto out = m_outExtend();
            auto in = m_inExtend();

            auto extend = out >= in ? out : -in;
            m_climbSubsystem->RunMotorExternal(extend);
        }

        void End(bool interrupted) override {
            m_climbSubsystem->StopMovement();
        }

    private:
        bool isFinished;
        ClimbSubsystem *m_climbSubsystem;
        std::function<double()> m_inExtend;
        std::function<double()> m_outExtend;
};