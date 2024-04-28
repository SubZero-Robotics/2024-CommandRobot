// See
// https://github.com/frc1675/frc1675-2024/blob/42d94881e7dd001fac9bb410892267b4d4dd8063/src/main/java/frc/robot/util/ChangableSendableChooser.java

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

#include <atomic>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <ranges>
#include <string>
#include <vector>

template <typename T>
class ModifiableChooser : public wpi::Sendable {
 private:
  static inline const std::string kDefault = "default";
  static inline const std::string kSelected = "selected";
  static inline const std::string kActive = "active";
  static inline const std::string kOptions = "options";
  static inline const std::string kInstance = ".instance";

  std::map<std::string, T> m_map;

  std::string m_defaultChoice = "";
  int m_instance;
  std::string m_previousValue;
  std::function<void(std::optional<T>)> m_listener;
  std::atomic_int m_instances{0};
  std::string m_selected;
  std::recursive_mutex m_mutex;

 public:
  ModifiableChooser() {
    m_instance = m_instances.fetch_add(1);
    wpi::SendableRegistry::Add(this, "SendableChangableChooser", m_instance);
  }

  ~ModifiableChooser() { wpi::SendableRegistry::Remove(this); }

  void AddOption(std::string name, T object) { m_map[name] = object; }

  void RemoveOption(std::string name) {
    if (m_map.contains(name)) {
      if (m_defaultChoice == name) {
        m_defaultChoice = "";
      }

      if (m_selected == name) {
        m_selected = m_defaultChoice;
      }

      m_map.erase(name);
    }
  }

  void ClearOptions() {
    m_defaultChoice = "";
    m_selected = m_defaultChoice;

    m_map.clear();
  }

  void SetOptions(std::map<std::string, T> options) {
    ClearOptions();

    m_map = options;
  }

  void SetDefaultOption(std::string name, T object) {
    m_defaultChoice = name;
    AddOption(name, object);
  }

  void SetOptions(std::map<std::string, T> options, std::string defaultName,
                  T defaultObject) {
    SetOptions(options);
    SetDefaultOption(defaultName, defaultObject);
    if (m_selected == "") {
      m_selected = defaultName;
    }
  }

  T GetSelected() {
    std::lock_guard<std::recursive_mutex> lk(m_mutex);

    try {
      // TODO
      if (m_selected != "") {
        return m_map[m_selected];
      }

      return m_map[m_defaultChoice];
    } catch (const std::exception& ex) {
      std::cerr << ex.what() << '\n';
    }
  }

  std::string GetSelectedKey() {
    std::lock_guard<std::recursive_mutex> lk(m_mutex);

    try {
      // TODO
      if (m_selected != "") {
        return m_selected;
      }

      return m_defaultChoice;
    } catch (const std::exception& ex) {
      std::cerr << ex.what() << '\n';
    }
  }

  std::string GetNtSelected() {
    std::optional<T> choice;
    std::function<void(std::optional<T>)> listener;
    std::string setSelectedTo;

    std::lock_guard<std::recursive_mutex> lk(m_mutex);

    try {
      if (m_selected != "") {
        setSelectedTo = m_selected;
      } else {
        setSelectedTo = m_defaultChoice;
      }

      if (setSelectedTo != m_previousValue && m_listener &&
          m_map.contains(setSelectedTo)) {
        choice = m_map[setSelectedTo];
        listener = m_listener;
      } else {
        choice = std::nullopt;
        listener = nullptr;
      }

      m_previousValue = setSelectedTo;

      if (listener) {
        listener(choice);
      }
    } catch (const std::exception& ex) {
      std::cerr << ex.what() << '\n';
    }

    return setSelectedTo;
  }

  void SetNtSelected(std::string val) {
    std::optional<T> choice;
    std::function<void(std::optional<T>)> listener;

    std::lock_guard<std::recursive_mutex> lk(m_mutex);

    try {
      m_selected = val;

      if (!m_map.contains(m_selected)) {
        m_selected = m_defaultChoice;
      }

      if (m_selected != m_previousValue && m_listener) {
        choice = m_map[val];
        listener = m_listener;
      } else {
        choice = std::nullopt;
        listener = nullptr;
      }

      m_previousValue = val;

      if (listener) {
        listener(choice);
      }
    } catch (const std::exception& ex) {
      std::cerr << ex.what() << '\n';
    }
  }

  void OnChange(std::function<void(std::optional<T>)> listener) {
    std::lock_guard<std::recursive_mutex> lk(m_mutex);
    m_listener = listener;
  }

  void InitSendable(wpi::SendableBuilder& builder) override {
    builder.SetSmartDashboardType("String Chooser");
    builder.PublishConstInteger(kInstance, m_instance);
    builder.AddStringProperty(
        kDefault, [this] { return m_defaultChoice; }, nullptr);
    builder.AddStringArrayProperty(
        kOptions,
        [this] {
          auto keys = std::views::keys(m_map);
          return std::vector<std::string>{keys.begin(), keys.end()};
        },
        nullptr);
    builder.AddStringProperty(
        kActive, std::bind(&ModifiableChooser::GetSelectedKey, this), nullptr);
    builder.AddStringProperty(
        kSelected, std::bind(&ModifiableChooser::GetNtSelected, this),
        [this](std::string_view val) { SetNtSelected(std::string{val}); });
  }
};