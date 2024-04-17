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

#include "Constants.h"
#include "utils/ConsoleLogger.h"

namespace AutoFactory {
frc2::CommandPtr GetEmptyCommand() { return frc2::WaitCommand(15_s).ToPtr(); }

bool AutoFileExists(std::string fileName) {
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

frc2::CommandPtr PathPlannerPathFromName(std::string autoName) {
  if (!AutoFileExists(autoName)) {
    ConsoleWriter.logError(
        "Auto Factory", "AUTO '%s' DOES NOT EXIST HELP US EVAN",
        autoName.c_str());
    return GetEmptyCommand();
  }
  return pathplanner::PathPlannerAuto(autoName).ToPtr();
}

frc2::CommandPtr GetAuto(AutoConstants::AutoType type) {
  using namespace AutoConstants;

  switch (type) {
    case AutoType::EmptyAuto:
      return GetEmptyCommand();
    case AutoType::FourNoteAuto:
      return PathPlannerPathFromName("4 Note Auto");
    case AutoType::PlaceAndLeave:
      return PathPlannerPathFromName("Place and leave");
    case AutoType::ThreeNoteAuto:
      return PathPlannerPathFromName("3 Note Auto");
    case AutoType::TwoNoteAuto:
      return PathPlannerPathFromName("2 Note Amp Side");
    case AutoType::TwoNoteCenter:
      return PathPlannerPathFromName("2 Note Center Note 3");
    case AutoType::TwoNoteSource:
      return PathPlannerPathFromName("2 Note Source Side");
    case AutoType::ThreeNoteCenter:
      return PathPlannerPathFromName("3 Note Center Note 3 + 4");
    case AutoType::LeaveWing:
      return PathPlannerPathFromName(AutoConstants::kDefaultAutoName);
    default:
      ConsoleWriter.logWarning(
          "Auto Factory",
          "Auto type %d does not exist, defaulting to empty "
          "auto",
          static_cast<int>(type));
      return GetEmptyCommand();
  }
}
}  // namespace AutoFactory