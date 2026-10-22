#pragma once
#include <stdexcept>
#include <string>
#include <vector>

namespace orchestrator {
// Provisional dependencies from PRD section 3 (2026 reference, not 2027 approval).
inline void validate_prerequisites(const std::vector<std::string> &tasks) {
  if (tasks.empty())
    throw std::invalid_argument("mission plan is empty");
  bool navigation = false, acquired = false, returned = false;
  for (const auto &task : tasks) {
    if (task == "PASS_GATE" || task == "TURN" || task == "HOLD" ||
        task == "DIVE_TO_DEPTH" || task == "APPROACH_GATE")
      continue;
    if (task == "NAVIGATION") {
      navigation = true;
      continue;
    }
    if (task != "TARGET_ACQUISITION" && task != "RETURN_GATE" && task != "TARGET_REACQUISITION" &&
        task != "COMMUNICATION")
      throw std::invalid_argument("unknown task: " + task);
    if (!navigation)
      throw std::invalid_argument(task + " requires NAVIGATION");
    if (task == "TARGET_ACQUISITION") {
      acquired = true;
      returned = false;
    }
    if (task == "RETURN_GATE") {
      if (!acquired)
        throw std::invalid_argument("RETURN_GATE requires TARGET_ACQUISITION");
      returned = true;
    }
    if (task == "TARGET_REACQUISITION" && (!acquired || !returned))
      throw std::invalid_argument(
          "TARGET_REACQUISITION requires acquisition and a subsequent RETURN_GATE");
  }
}

inline void require_supported_tasks(const std::vector<std::string> &tasks, bool hardware = false) {
  for (const auto &task : tasks)
    if (hardware ? task != "DIVE_TO_DEPTH" && task != "TURN" && task != "APPROACH_GATE" :
        task != "PASS_GATE" && task != "TURN" && task != "HOLD" && task != "NAVIGATION")
      throw std::invalid_argument(
          task + (hardware ? ": unavailable in the onboard hardware profile" :
              ": unavailable until payload/localization hardware and final rules are verified"));
}
}  // namespace orchestrator
