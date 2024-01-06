#include "utils/ShuffleboardLogger.h"
#include <wpi/json.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace Logging;

ShuffleboardLogger::ShuffleboardLogger() {
  
}

void ShuffleboardLogger::logVerbose(std::string key, const std::string format, ...) {
  if (!shouldLog(LogLevel::VERBOSE)) return;

  va_list args;
  va_start(args, format);
  frc::SmartDashboard::PutString(key, formatToShuffleboardString(LogLevel::VERBOSE, format, args));
  va_end(args);
}
void ShuffleboardLogger::logInfo(std::string key, const std::string format, ...) {
  if (!shouldLog(LogLevel::INFO)) return;

  va_list args;
  va_start(args, format);
  frc::SmartDashboard::PutString(key, formatToShuffleboardString(LogLevel::INFO, format, args));
  va_end(args);
}
void ShuffleboardLogger::logWarning(std::string key, const std::string format, ...) {
  if (!shouldLog(LogLevel::WARNING)) return;

  va_list args;
  va_start(args, format);
  frc::SmartDashboard::PutString(key, formatToShuffleboardString(LogLevel::WARNING, format, args));
  va_end(args);
}
void ShuffleboardLogger::logError(std::string key, const std::string format, ...) {
  if (!shouldLog(LogLevel::ERROR)) return;

  va_list args;
  va_start(args, format);
  frc::SmartDashboard::PutString(key, formatToShuffleboardString(LogLevel::ERROR, format, args));
  va_end(args);
}
void ShuffleboardLogger::logFatal(std::string key, const std::string format, ...) {
  if (!shouldLog(LogLevel::FATAL)) return;

  va_list args;
  va_start(args, format);
  frc::SmartDashboard::PutString(key, formatToShuffleboardString(LogLevel::FATAL, format, args));
  va_end(args);
}

void ShuffleboardLogger::logInfo(std::string key, int val) {
  if (!shouldLog(LogLevel::INFO)) return;

  frc::SmartDashboard::PutNumber(key, val);
}
void ShuffleboardLogger::logVerbose(std::string key, int val) {
  if (!shouldLog(LogLevel::VERBOSE)) return;

  frc::SmartDashboard::PutNumber(key, val);
}
void ShuffleboardLogger::logWarning(std::string key, int val) {
  if (!shouldLog(LogLevel::WARNING)) return;

  frc::SmartDashboard::PutNumber(key, val);
}
void ShuffleboardLogger::logError(std::string key, int val) {
  if (!shouldLog(LogLevel::ERROR)) return;

  frc::SmartDashboard::PutNumber(key, val);
}
void ShuffleboardLogger::logFatal(std::string key, int val) {
  if (!shouldLog(LogLevel::FATAL)) return;

  frc::SmartDashboard::PutNumber(key, val);
}

void ShuffleboardLogger::logInfo(std::string key, double val) {
  if (!shouldLog(LogLevel::INFO)) return;

  frc::SmartDashboard::PutNumber(key, val);
}
void ShuffleboardLogger::logVerbose(std::string key, double val) {
  if (!shouldLog(LogLevel::VERBOSE)) return;

  frc::SmartDashboard::PutNumber(key, val);
}
void ShuffleboardLogger::logWarning(std::string key, double val) {
  if (!shouldLog(LogLevel::WARNING)) return;

  frc::SmartDashboard::PutNumber(key, val);
}
void ShuffleboardLogger::logError(std::string key, double val) {
  if (!shouldLog(LogLevel::ERROR)) return;

  frc::SmartDashboard::PutNumber(key, val);
}
void ShuffleboardLogger::logFatal(std::string key, double val) {
  if (!shouldLog(LogLevel::FATAL)) return;

  frc::SmartDashboard::PutNumber(key, val);
}

void ShuffleboardLogger::logInfo(std::string key, bool val) {
  if (!shouldLog(LogLevel::INFO)) return;

  frc::SmartDashboard::PutBoolean(key, val);
}
void ShuffleboardLogger::logVerbose(std::string key, bool val) {
  if (!shouldLog(LogLevel::VERBOSE)) return;

  frc::SmartDashboard::PutBoolean(key, val);
}
void ShuffleboardLogger::logWarning(std::string key, bool val) {
  if (!shouldLog(LogLevel::WARNING)) return;

  frc::SmartDashboard::PutBoolean(key, val);
}
void ShuffleboardLogger::logError(std::string key, bool val) {
  if (!shouldLog(LogLevel::ERROR)) return;

  frc::SmartDashboard::PutBoolean(key, val);
}
void ShuffleboardLogger::logFatal(std::string key, bool val) {
  if (!shouldLog(LogLevel::FATAL)) return;

  frc::SmartDashboard::PutBoolean(key, val);
}

void ShuffleboardLogger::logInfo(std::string key, frc::Pose2d& val) {
  if (!shouldLog(LogLevel::INFO)) return;

  frc::SmartDashboard::PutString(key, poseToString(val));
}
void ShuffleboardLogger::logVerbose(std::string key, frc::Pose2d& val) {
  if (!shouldLog(LogLevel::VERBOSE)) return;

  frc::SmartDashboard::PutString(key, poseToString(val));
}
void ShuffleboardLogger::logWarning(std::string key, frc::Pose2d& val) {
  if (!shouldLog(LogLevel::WARNING)) return;

  frc::SmartDashboard::PutString(key, poseToString(val));
}
void ShuffleboardLogger::logError(std::string key, frc::Pose2d& val) {
  if (!shouldLog(LogLevel::ERROR)) return;

  frc::SmartDashboard::PutString(key, poseToString(val));
}
void ShuffleboardLogger::logFatal(std::string key, frc::Pose2d& val) {
  if (!shouldLog(LogLevel::FATAL)) return;

  frc::SmartDashboard::PutString(key, poseToString(val));
}

void ShuffleboardLogger::logInfo(std::string key, wpi::Sendable* val) {
  if (!shouldLog(LogLevel::INFO)) return;

  frc::SmartDashboard::PutData(key, val);
}
void ShuffleboardLogger::logVerbose(std::string key, wpi::Sendable* val) {
  if (!shouldLog(LogLevel::VERBOSE)) return;

  frc::SmartDashboard::PutData(key, val);
}
void ShuffleboardLogger::logWarning(std::string key, wpi::Sendable* val) {
  if (!shouldLog(LogLevel::WARNING)) return;

  frc::SmartDashboard::PutData(key, val);
}
void ShuffleboardLogger::logError(std::string key, wpi::Sendable* val) {
  if (!shouldLog(LogLevel::ERROR)) return;

  frc::SmartDashboard::PutData(key, val);
}
void ShuffleboardLogger::logFatal(std::string key, wpi::Sendable* val) {
  if (!shouldLog(LogLevel::FATAL)) return;

  frc::SmartDashboard::PutData(key, val);
}