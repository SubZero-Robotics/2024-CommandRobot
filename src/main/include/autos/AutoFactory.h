#pragma once

#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/WaitCommand.h>
#include <pathplanner/lib/commands/FollowPathHolonomic.h>
#include <wpi/MemoryBuffer.h>

#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "Constants.h"
#include "utils/ConsoleLogger.h"

template <typename T>
class AutoFactory {
 public:
  AutoFactory(const std::map<T, std::string>& autos) : m_autos{autos} {}

 private:
  const std::map<T, std::string>& m_autos;

  inline frc2::CommandPtr GetEmptyCommand() {
    return frc2::WaitCommand(15_s).ToPtr();
  }

  bool AutoFileExists(const std::string fileName) {
    const std::string filePath = frc::filesystem::GetDeployDirectory() +
                                 "/pathplanner/autos/" + fileName + ".auto";

    std::error_code error_code;
    std::unique_ptr<wpi::MemoryBuffer> fileBuffer =
        wpi::MemoryBuffer::GetFile(filePath, error_code);

    if (fileBuffer == nullptr || error_code) {
      return false;
    }

    return true;
  }

  frc2::CommandPtr PathPlannerPathFromName(const std::string autoName) {
    if (!AutoFileExists(autoName)) {
      ConsoleWriter.logError("Auto Factory",
                             "AUTO '%s' DOES NOT EXIST HELP US EVAN",
                             autoName.c_str());
      return GetEmptyCommand();
    }
    return pathplanner::PathPlannerAuto(autoName).ToPtr();
  }

 public:
  frc2::CommandPtr GetAuto(T type) {
    using namespace AutoConstants;

    if (!m_autos.contains(type)) {
      ConsoleWriter.logWarning(
          "Auto Factory",
          "Auto type %d does not exist, defaulting to empty "
          "auto",
          static_cast<int>(type));
      return GetEmptyCommand();
    }

    return PathPlannerPathFromName(m_autos.at(type));
  }
};