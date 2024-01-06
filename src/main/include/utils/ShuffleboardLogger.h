#ifndef SHUFFLEBOARD_LOGGER_H
#define SHUFFLEBOARD_LOGGER_H

#include <iostream>

#include "ILogger.h"

using namespace Logging;

class ShuffleboardLogger : ILogger {
  public:
    ShuffleboardLogger();

    void logVerbose(std::string key, const std::string format, ...) override;
    void logInfo(std::string key, const std::string format, ...) override;
    void logWarning(std::string key, const std::string format, ...) override;
    void logError(std::string key, const std::string format, ...) override;
    void logFatal(std::string key, const std::string format, ...) override;

    void logInfo(std::string key, int val) override;
    void logVerbose(std::string key, int val) override;
    void logWarning(std::string key, int val) override;
    void logError(std::string key, int val) override;
    void logFatal(std::string key, int val) override;

    void logInfo(std::string key, double val) override;
    void logVerbose(std::string key, double val) override;
    void logWarning(std::string key, double val) override;
    void logError(std::string key, double val) override;
    void logFatal(std::string key, double val) override;

    void logInfo(std::string key, bool val) override;
    void logVerbose(std::string key, bool val) override;
    void logWarning(std::string key, bool val) override;
    void logError(std::string key, bool val) override;
    void logFatal(std::string key, bool val) override;

    void logInfo(std::string key, frc::Pose2d& val) override;
    void logVerbose(std::string key, frc::Pose2d& val) override;
    void logWarning(std::string key, frc::Pose2d& val) override;
    void logError(std::string key, frc::Pose2d& val) override;
    void logFatal(std::string key, frc::Pose2d& val) override;

    void logInfo(std::string key, wpi::Sendable* val) override;
    void logVerbose(std::string key, wpi::Sendable* val) override;
    void logWarning(std::string key, wpi::Sendable* val) override;
    void logError(std::string key, wpi::Sendable* val) override;
    void logFatal(std::string key, wpi::Sendable* val) override;

  private:
    std::string formatToShuffleboardString(LogLevel level, const std::string format, ...) {
      va_list args;
      va_start(args, format);
      std::string val = formatString(format, args);
      val = levelToString(level) + " - " + val;
      va_end(args);

      return val;
    }
};

static ShuffleboardLogger shuffleboardLogger;

#endif