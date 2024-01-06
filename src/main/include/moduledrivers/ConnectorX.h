#pragma once

#include <frc/I2C.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/util/Color8Bit.h>

#include <memory>

#include "Constants.h"

namespace ConnectorX {
struct Message {
  uint8_t data[61];
  uint8_t len;
  // Set to 0xFFFF to send to all
  uint16_t teamNumber;
};
namespace Commands {
struct LedConfiguration {
  uint16_t count;
  uint8_t brightness;
};

struct Configuration {
  int8_t valid;
  uint16_t teamNumber;
  // Send messages to 2 other teams
  uint16_t initialTeams[2];
  uint8_t i2c0Addr;
  LedConfiguration led0;
  LedConfiguration led1;
};
enum class CommandType {
  // W
  On = 0,
  // W
  Off = 1,
  // W
  Pattern = 2,
  // W
  ChangeColor = 3,
  // R
  ReadPatternDone = 4,
  // W
  SetLedPort = 5,
  // R
  ReadAnalog = 6,
  // W
  DigitalSetup = 7,
  // W
  DigitalWrite = 8,
  // R
  DigitalRead = 9,
  // W
  SetConfig = 10,
  // R
  ReadConfig = 11,
  // W
  RadioSend = 12,
  // R
  RadioGetLatestReceived = 13
};

struct CommandOn {};

struct CommandOff {};

// * Set delay to -1 to use default delay
struct CommandPattern {
  uint8_t pattern;
  uint8_t oneShot;
  int16_t delay;
};

struct CommandColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

struct CommandReadPatternDone {};

struct CommandSetLedPort {
  uint8_t port;
};

struct CommandReadAnalog {
  uint8_t port;
};

struct CommandDigitalSetup {
  uint8_t port;
  /** Follows the Arduino-defined values for pinMode
   * INPUT = 0
   * INPUT_PULLUP = 2
   * INPUT_PULLDOWN = 3
   * OUTPUT = 1
   * OUTPUT_2MA = 4
   * OUTPUT_4MA = 5
   * OUTPUT_8MA = 6
   * OUTPUT_12MA = 7
   */
  uint8_t mode;
};

struct CommandDigitalWrite {
  uint8_t port;
  uint8_t value;
};

struct CommandDigitalRead {
  uint8_t port;
};

struct CommandSetConfig {
  Configuration config;
};

struct CommandReadConfig {};

struct CommandRadioSend {
  Message msg;
};

struct CommandRadioGetLatestReceived {};

union CommandData {
  CommandOn commandOn;
  CommandOff commandOff;
  CommandPattern commandPattern;
  CommandColor commandColor;
  CommandReadPatternDone commandReadPatternDone;
  CommandSetLedPort commandSetLedPort;
  CommandReadAnalog commandReadAnalog;
  CommandDigitalSetup commandDigitalSetup;
  CommandDigitalWrite commandDigitalWrite;
  CommandDigitalRead commandDigitalRead;
  CommandSetConfig commandSetConfig;
  CommandReadConfig commandReadConfig;
  CommandRadioSend commandRadioSend;
  CommandRadioGetLatestReceived commandRadioGetLatestReceived;
};

struct Command {
  CommandType commandType;
  CommandData commandData;
};

struct ResponsePatternDone {
  uint8_t done;
};

struct ResponseReadAnalog {
  uint16_t value;
};

struct ResponseDigitalRead {
  uint8_t value;
};

struct ResponseRadioLastReceived {
  Message msg;
};

struct ResponseReadConfiguration {
  Commands::Configuration config;
};

union ResponseData {
  ResponsePatternDone responsePatternDone;
  ResponseReadAnalog responseReadAnalog;
  ResponseDigitalRead responseDigitalRead;
  ResponseRadioLastReceived responseRadioLastReceived;
  ResponseReadConfiguration responseReadConfiguration;
};

struct Response {
  CommandType commandType;
  ResponseData responseData;
};
}  // namespace Commands
enum class PatternType {
  None = 0,
  SetAll = 1,
  Blink = 2,
  RGBFade = 3,
  HackerMode = 4
};

enum class PinMode {
  INPUT = 0,
  OUTPUT = 1,
  INPUT_PULLUP = 2,
  INPUT_PULLDOWN = 3,
  OUTPUT_2MA = 4,
  OUTPUT_4MA = 5,
  OUTPUT_8MA = 6,
  OUTPUT_12MA = 7
};

enum class DigitalPort { D0 = 0, D1 = 1, D2 = 2, D3 = 3, D4 = 4, D5 = 5 };

enum class AnalogPort { A0 = 0, A1 = 1, A2 = 2 };

enum class LedPort { P0 = 0, P1 = 1 };

class ConnectorXBoard : public frc2::SubsystemBase {
 public:
  ConnectorXBoard(uint8_t slaveAddress, frc::I2C::Port port = frc::I2C::kMXP);

  /**
   * @brief Start communication with the controller
   *
   * @return true if successful init
   */
  bool initialize();

  /**
   * @brief Get the last command sent
   *
   * @return CommandType
   */
  inline Commands::CommandType lastCommand() const { return _lastCommand; }

  /**
   * @brief Get the last pattern sent
   *
   * @param port
   * @return PatternType
   */
  inline PatternType lastPattern(LedPort port) const {
    return _lastPattern[(uint8_t)port];
  }

  /**
   * @brief Configure a digital IO pin. Think Arduino pinMode()
   *
   * @param port
   * @param mode Input, Output, etc.
   */
  void configureDigitalPin(DigitalPort port, PinMode mode);

  /**
   * @brief Set a digital IO pin
   *
   * @param value
   */
  void writeDigitalPin(DigitalPort port, bool value);

  /**
   * @brief Read state of digital IO pin
   *
   * @param port
   * @return true - Remember this will be HIGH by default if set to INPUT_PULLUP
   * @return false
   */
  bool readDigitalPin(DigitalPort port);

  /**
   * @brief Read the ADC value. Ranges from 0 - 3.3v; 12 bits of precision
   *
   * @param port
   * @return uint16_t
   */
  uint16_t readAnalogPin(AnalogPort port);

  /**
   * @brief Turn on
   *
   */
  void setOn(LedPort port);

  /**
   * @brief Turn off
   *
   */
  void setOff(LedPort port);

  /**
   * @brief Send the PATTERN command
   *
   * @param pattern
   * @param oneShot Only run the pattern once
   */
  void setPattern(LedPort port, PatternType pattern, bool oneShot = false,
                  int16_t delay = -1);

  /**
   * @brief Set the color; must also call a pattern to see it
   *
   */
  void setColor(LedPort port, uint8_t red, uint8_t green, uint8_t blue);

  /**
   * @brief Set the color using frc::Color8Bit
  */
  void setColor(LedPort port, frc::Color8Bit color) {
    setColor(port, color.red, color.green, color.blue);
  };
  /**
   * @brief Set the color; must also call a pattern to see it
   *
   * @param color Color data in the form of 0x00RRGGBB
   */
  void setColor(LedPort port, uint32_t color);

  bool compareColor(frc::Color8Bit C1, frc::Color8Bit C2) {
    return ((C1.red == C2.red) && (C1.blue = C2.blue) && (C1.green == C2.green));
  }

  /**
   * @brief Read if pattern is done running
   *
   * @return true if pattern is done
   */
  bool getPatternDone(LedPort port);

  /**
   * @brief Store the config in board's EEPROM
   *
   * @param config
   */
  void setConfig(Commands::Configuration config);

  /**
   * @brief Read the config stored in EEPROM
   *
   * @return Configuration
   */
  Commands::Configuration readConfig();

  /**
   * @brief Send a message via the radio; set team to 0xFFFF to broadcast to all
   *
   * @param message
   */
  void sendRadioMessage(Message message);

  inline frc::Color8Bit getCurrentColor(LedPort port) {
    return m_currentColors[(uint8_t)port];
  }

  /**
   * @brief Read the last received message
   *
   * @return Message
   */
  Message getLatestRadioMessage();

 private:
  Commands::Response sendCommand(Commands::Command command,
                                 bool expectResponse = false);

  void setLedPort(LedPort port);

  std::unique_ptr<frc::I2C> _i2c;
  uint8_t _slaveAddress;
  LedPort _currentLedPort = LedPort::P0;
  Commands::CommandType _lastCommand;
  PatternType _lastPattern[2];
  frc::Color8Bit m_currentColors[2] = { {0, 0, 0}, {0, 0, 0}};
};
}  // namespace ConnectorX
