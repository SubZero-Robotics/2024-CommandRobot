#include "utils/ConsoleLogger.h"

int main(int argc, char** argv) {
  auto& instance = ConsoleWriter;
  instance.logInfo("key", "format string %s %d %u\n", "test", 123, 12u);
  return 0;
}
