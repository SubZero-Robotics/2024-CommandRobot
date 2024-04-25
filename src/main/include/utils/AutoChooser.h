#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "utils/ModifiableChooser.h"

// Each key of type T has a vector<string> of tags
// Construct groups with a name and list of possible tags that are mutually
// exclusive, including ANY as an option When a selection is made, grab the
// intersection of T where all selections are in T's list of tags

template <typename TKey>
class AutoChooser {
 public:
  // A pair of:
  // - The entry's key
  // - The entry's name
  using AutoChooserValue = std::pair<TKey, std::string>;
  // A pair of:
  // - The entry's key and name
  // - A set of associated tags
  using AutoChooserEntry = std::pair<AutoChooserValue, std::set<std::string>>;
  // A pair of:
  // - The group's name
  // - A set of associated tags that can be selected from
  using AutoChooserSelectorGroup =
      std::pair<std::string, std::set<std::string>>;

  AutoChooser(const std::vector<AutoChooserEntry>& entries,
              const std::vector<AutoChooserSelectorGroup>& groups,
              std::string chooserName)
      : m_chooserName{chooserName} {
    m_entries = entries;
    m_groups.reserve(groups.size());

    for (auto& group : groups) {
      m_groups.push_back({
          .group = group,
          .chooser = std::make_unique<frc::SendableChooser<std::string>>(),
      });
    }

    m_chooser.OnChange([this](std::optional<TKey> value) {
      if (value && m_onChangeCb) {
        m_onChangeCb(value.value());
      }
    });

    frc::SmartDashboard::PutData(m_chooserName, &m_chooser);
  }

  // Pushes everything to smart dashboard
  void Initialize() {
    for (auto& group : m_groups) {
      for (auto& option : group.group.second) {
        group.chooser->AddOption(option, option);
      }

      group.chooser->SetDefaultOption("ANY", "ANY");
      group.chooser->OnChange([this](std::string newValue) {
        auto availableEntries = GetAvailableEntries();
        PopulateChooser();
        std::vector<TKey> availableKeys;
        availableKeys.reserve(availableEntries.size());

        std::transform(
            availableEntries.begin(), availableEntries.end(),
            std::back_inserter(availableKeys),
            [](const AutoChooserValue& value) { return value.first; });
      });

      frc::SmartDashboard::PutData(group.group.first, group.chooser.get());
    }

    PopulateChooser();
  }

  // Will be called any time the available entries change
  void SetOnChangeCallback(std::function<void(TKey)> cb) { m_onChangeCb = cb; }

  // Returns a list of all valid entries by key and name
  std::vector<AutoChooserValue> GetAvailableEntries() {
    std::vector<AutoChooserValue> availableEntries;
    std::vector<std::string> selectedTags;
    for (auto& group : m_groups) {
      auto selected = group.chooser->GetSelected();
      if (selected != "ANY") {
        selectedTags.push_back(selected);
      }
    }

    for (auto& entry : m_entries) {
      bool matches = true;
      for (auto& tag : selectedTags) {
        if (!entry.second.contains(tag)) {
          matches = false;
          break;
        }
      }

      if (matches) {
        availableEntries.push_back(entry.first);
      }
    }

    return availableEntries;
  }

  void PopulateChooser() {
    auto entries = GetAvailableEntries();
    m_chooser.ClearOptions();

    for (auto it = entries.begin(); it != entries.end(); it++) {
      if (it == entries.begin()) {
        m_chooser.SetDefaultOption(it->second, it->first);
        continue;
      }

      m_chooser.AddOption(it->second, it->first);
    }
  }

  inline TKey GetSelectedValue() { return m_chooser.GetSelected(); }

 private:
  struct AutoChooserSendableGroup {
    AutoChooserSelectorGroup group;
    std::unique_ptr<frc::SendableChooser<std::string>> chooser;
  };
  std::function<void(TKey)> m_onChangeCb;
  std::vector<AutoChooserEntry> m_entries;
  std::vector<AutoChooserSendableGroup> m_groups;
  ModifiableChooser<TKey> m_chooser;
  std::string m_chooserName;
};