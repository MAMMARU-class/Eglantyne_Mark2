#ifndef EXPERIMENTAL_SETUP_H
#define EXPERIMENTAL_SETUP_H

#include <array>

// constexpr float EXPERIMENT_T_SUP_INITIAL = 0.14f;
// constexpr float EXPERIMENT_T_SUP_FINAL = 0.20f;

constexpr float EXPERIMENT_T_SUP_INITIAL = 0.22f;
constexpr float EXPERIMENT_T_SUP_FINAL = 0.22f;
constexpr std::size_t EXPERIMENT_T_SUP_HOLD_STEP_COUNT = 100;

enum class FeedbackGainMode {
    FIXED_MAXIMUM,
    T_SUP_DEPENDENT,
    FIXED_MINIMUM
};

constexpr FeedbackGainMode EXPERIMENT_FEEDBACK_GAIN_MODE =
    FeedbackGainMode::FIXED_MAXIMUM;

constexpr bool EXPERIMENT_DISTURBANCE_ENABLED = false;
constexpr std::array<float, 3> TARGET_VELOCITY = {0.05f, 0.0f, 0.0f};
constexpr float VELOCITY_EPS = 1e-4f;

constexpr std::size_t EXPERIMENT_LOG_ROW_COUNT = 3000;

static_assert(
    EXPERIMENT_LOG_ROW_COUNT > EXPERIMENT_T_SUP_HOLD_STEP_COUNT,
    "The log must contain steps after the fixed T_sup interval");

#endif
