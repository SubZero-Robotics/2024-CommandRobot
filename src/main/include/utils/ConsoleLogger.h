#ifndef CONSOLE_LOGGER_H
#define CONSOLE_LOGGER_H

#include <iostream>
#include <string>

#include "ILogger.h"

class ConsoleLogger : ILogger {
 public:
  static ConsoleLogger& getInstance() {
    static ConsoleLogger instance;

    return instance;
  }

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
  ConsoleLogger();

  void log(Logging::LogLevel level, std::string key, std::string fmt,
           va_list ap) {
    if (!shouldLog(level)) return;

    std::string val = formatString(fmt, ap);
    std::cout << levelToString(level) << " - " << key << ": " << val
              << std::endl;
  }
};

#endif