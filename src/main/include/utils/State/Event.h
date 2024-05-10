#pragma once

namespace StateManager {
template <typename TKey, typename TStateKey>
struct Event {
  Tkey key;
  TStateKey stateKey;
};
}  // namespace StateManager