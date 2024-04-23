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

// Each key of type T has a vector<string> of tags
// Construct groups with a name and list of possible tags that are mutually
// exclusive, including ANY as an option When a selection is made, grab the
// intersection of T where all selections are in T's list of tags

// A pair of:
// - The entry's key
// - The entry's name
template <typename TKey>
using AutoChooserValue = std::pair<TKey, std::string>;
// A pair of:
// - The entry's key and name
// - A set of associated tags
template <typename TKey>
using AutoChooserEntry =
    std::pair<AutoChooserValue<TKey>, std::set<std::string>>;
// A pair of:
// - The group's name
// - A set of associated tags that can be selected from
using AutoChooserSelectorGroup = std::pair<std::string, std::set<std::string>>;

template <typename TKey>
class AutoChooser {
 public:
  AutoChooser(const std::vector<AutoChooserEntry<TKey>>& entries,
              const std::vector<AutoChooserSelectorGroup>& groups,
              std::string chooserName) {
    m_entries = entries;
    m_groups.reserve(groups.size());

    for (auto& group : groups) {
      m_groups.push_back({
          .group = group,
          .chooser = std::make_unique<frc::SendableChooser<std::string>>(),
      });
    }
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
            [](const AutoChooserValue<TKey>& value) { return value.first; });

        if (m_onChangeCb) {
          m_onChangeCb(availableKeys);
        }
      });
      frc::SmartDashboard::PutData(group.group.first, group.chooser.get());
    }

    PopulateChooser();
  }

  // Will be called any time the available entries change
  void SetOnChangeCallback(std::function<void(std::vector<TKey>)> cb) {
    m_onChangeCb = cb;
  }

  // Returns a list of all valid entries by key and name
  std::vector<AutoChooserValue<TKey>> GetAvailableEntries() {
    std::vector<AutoChooserValue<TKey>> availableEntries;
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
    m_chooser = std::make_unique<frc::SendableChooser<TKey>>();

    for (auto& entry : entries) {
      m_chooser->AddOption(entry.second, entry.first);
    }

    // This causes a crash after the initial creation of the new chooser...
    // ? Better way to clear and repopulate the values?
    frc::SmartDashboard::PutData(m_chooserName, m_chooser.get());
  }

  inline TKey GetSelectedValue() { return m_chooser->GetSelected(); }

 private:
  struct AutoChooserSendableGroup {
    AutoChooserSelectorGroup group;
    std::unique_ptr<frc::SendableChooser<std::string>> chooser;
  };
  std::function<void(std::vector<TKey>)> m_onChangeCb;
  std::vector<AutoChooserEntry<TKey>> m_entries;
  std::vector<AutoChooserSendableGroup> m_groups;
  std::unique_ptr<frc::SendableChooser<TKey>> m_chooser;
  std::string m_chooserName;
};