#include <gtest/gtest.h>
#include "orchestrator/plan.hpp"
using orchestrator::validate_prerequisites;
using orchestrator::require_supported_tasks;

TEST(Plan, QualificationAndUnknown) {
  EXPECT_NO_THROW(validate_prerequisites({"PASS_GATE", "TURN", "PASS_GATE"}));
  EXPECT_THROW(validate_prerequisites({}), std::invalid_argument);
  EXPECT_THROW(validate_prerequisites({"TYPO"}), std::invalid_argument);
}
TEST(Plan, FinalsRequireNavigation) {
  EXPECT_THROW(validate_prerequisites({"TARGET_ACQUISITION"}), std::invalid_argument);
  EXPECT_THROW(validate_prerequisites({"PASS_GATE", "COMMUNICATION"}), std::invalid_argument);
  EXPECT_NO_THROW(validate_prerequisites({"NAVIGATION", "COMMUNICATION"}));
}
TEST(Plan, ReacquisitionNeedsAcquisitionAndLaterGate) {
  EXPECT_THROW(validate_prerequisites({"NAVIGATION", "RETURN_GATE"}), std::invalid_argument);
  EXPECT_THROW(validate_prerequisites({"NAVIGATION", "TARGET_ACQUISITION", "TARGET_REACQUISITION"}), std::invalid_argument);
  EXPECT_NO_THROW(validate_prerequisites({"NAVIGATION", "TARGET_ACQUISITION", "RETURN_GATE", "TARGET_REACQUISITION"}));
  EXPECT_THROW(validate_prerequisites({"NAVIGATION", "TARGET_ACQUISITION", "RETURN_GATE", "TARGET_ACQUISITION", "TARGET_REACQUISITION"}), std::invalid_argument);
}
TEST(Plan, UnsupportedHardwareNeverPretendsToComplete) {
  EXPECT_NO_THROW(require_supported_tasks({"NAVIGATION", "TURN", "PASS_GATE"}));
  EXPECT_THROW(require_supported_tasks({"NAVIGATION", "TARGET_ACQUISITION"}), std::invalid_argument);
  EXPECT_THROW(require_supported_tasks({"NAVIGATION", "COMMUNICATION"}), std::invalid_argument);
}
