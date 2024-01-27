#include "2024-CommandRobot/src/main/include/utils/ConsoleLogger.h"

int main(int argc, char** argv) {
  auto& instance = ConsoleLogger::getInstance();
  instance.logInfo("key", "format string %s %d %u\n", "test", 123, 12u);
  return 0;
}
