#pragma once

#include <frc2/command/CommandPtr.h>
#include <units/time.h>

#include "utils/State/Event.h"

namespace StateManager {
template <typename TKey>
class State {
 public:
  // Next State??????
  explicit State(TKey key, frc2::CommandPtr&& command, units::second_t timeout,
                 std::optional<std::function<bool()>> interrupt,
                 std::vector<Event> events, std::function<void()> onEnd);

  const units::second_t GetTimeout() const {
    return m_timeout;
  }

 private:
  TKey m_key;
  frc2::CommandPtr m_command;
  units::second_t m_timeout;
  std::function<bool()> m_interrupt;
  const std::vector<Event> m_events;
};
}  // namespace StateManager