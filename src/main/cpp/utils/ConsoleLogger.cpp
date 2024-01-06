#include "utils/ConsoleLogger.h"
#include <wpi/json.h>

using namespace Logging;

ConsoleLogger::ConsoleLogger() {
  std::cout << "INFO - ConsoleLogger: Console Logger initialized!" << std::endl;
}

void ConsoleLogger::logVerbose(std::string key, const std::string format, ...) {
  va_list args;
  va_start(args, format);
  log(LogLevel::VERBOSE, key, format, args);
  va_end(args);
}
void ConsoleLogger::logInfo(std::string key, const std::string format, ...) {
  va_list args;
  va_start(args, format);
  log(LogLevel::INFO, key, format, args);
  va_end(args);
}
void ConsoleLogger::logWarning(std::string key, const std::string format, ...) {
  va_list args;
  va_start(args, format);
  log(LogLevel::WARNING, key, format, args);
  va_end(args);
}
void ConsoleLogger::logError(std::string key, const std::string format, ...) {
  va_list args;
  va_start(args, format);
  log(LogLevel::ERROR, key, format, args);
  va_end(args);
}
void ConsoleLogger::logFatal(std::string key, const std::string format, ...) {
  va_list args;
  va_start(args, format);
  log(LogLevel::FATAL, key, format, args);
  va_end(args);
}

void ConsoleLogger::logInfo(std::string key, int val) {
  logInfo(key, "%d", val);
}
void ConsoleLogger::logVerbose(std::string key, int val) {
  logVerbose(key, "%d", val);
}
void ConsoleLogger::logWarning(std::string key, int val) {
  logWarning(key, "%d", val);
}
void ConsoleLogger::logError(std::string key, int val) {
  logError(key, "%d", val);
}
void ConsoleLogger::logFatal(std::string key, int val) {
  logFatal(key, "%d", val);
}

void ConsoleLogger::logInfo(std::string key, double val) {
  logInfo(key, "%.3f", val);
}
void ConsoleLogger::logVerbose(std::string key, double val) {
  logVerbose(key, "%.3f", val);
}
void ConsoleLogger::logWarning(std::string key, double val) {
  logWarning(key, "%.3f", val);
}
void ConsoleLogger::logError(std::string key, double val) {
  logError(key, "%.3f", val);
}
void ConsoleLogger::logFatal(std::string key, double val) {
  logFatal(key, "%.3f", val);
}

void ConsoleLogger::logInfo(std::string key, bool val) {
  logInfo(key, "%s", val ? "true" : "false");
}
void ConsoleLogger::logVerbose(std::string key, bool val) {
  logVerbose(key, "%s", val ? "true" : "false");
}
void ConsoleLogger::logWarning(std::string key, bool val) {
  logWarning(key, "%s", val ? "true" : "false");
}
void ConsoleLogger::logError(std::string key, bool val) {
  logError(key, "%s", val ? "true" : "false");
}
void ConsoleLogger::logFatal(std::string key, bool val) {
  logFatal(key, "%s", val ? "true" : "false");
}

void ConsoleLogger::logInfo(std::string key, frc::Pose2d& val) {
  logInfo(key, poseToString(val));
}
void ConsoleLogger::logVerbose(std::string key, frc::Pose2d& val) {
  logVerbose(key, poseToString(val));
}
void ConsoleLogger::logWarning(std::string key, frc::Pose2d& val) {
  logWarning(key, poseToString(val));
}
void ConsoleLogger::logError(std::string key, frc::Pose2d& val) {
  logError(key, poseToString(val));
}
void ConsoleLogger::logFatal(std::string key, frc::Pose2d& val) {
  logFatal(key, poseToString(val));
}

void ConsoleLogger::logInfo(std::string key, wpi::Sendable* val) {
  logInfo(key, "Sendable");
}
void ConsoleLogger::logVerbose(std::string key, wpi::Sendable* val) {
  logVerbose(key, "Sendable");
}
void ConsoleLogger::logWarning(std::string key, wpi::Sendable* val) {
  logWarning(key, "Sendable");
}
void ConsoleLogger::logError(std::string key, wpi::Sendable* val) {
  logError(key, "Sendable");
}
void ConsoleLogger::logFatal(std::string key, wpi::Sendable* val) {
  logFatal(key, "Sendable");
}