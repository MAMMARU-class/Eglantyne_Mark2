#ifndef EXPERIMENTAL_SETUP_H
#define EXPERIMENTAL_SETUP_H

#include <array>

// options
// constexpr float EXPERIMENT_T_SUP_INITIAL = 0.14f;
// constexpr float EXPERIMENT_T_SUP_FINAL = 0.24f;

constexpr float EXPERIMENT_T_SUP_INITIAL = 0.14f;
constexpr float EXPERIMENT_T_SUP_FINAL = 0.24f;
constexpr std::size_t EXPERIMENT_T_SUP_HOLD_STEP_COUNT = 100;

enum class FeedbackGainMode {
    ZERO,
    FIXED_MAXIMUM,
    T_SUP_DEPENDENT,
    FIXED_MINIMUM
};

constexpr FeedbackGainMode EXPERIMENT_FEEDBACK_GAIN_MODE =
    FeedbackGainMode::ZERO;

constexpr bool EXPERIMENT_DISTURBANCE_ENABLED = false;

// values
struct UpdateRateFeedbackGains {
    float kp;
    float kd;
};

struct X0Vx0FeedbackGains {
    float kp;
    float kd;
};

constexpr UpdateRateFeedbackGains GAIN_ZERO = {
    0.0f,
    0.0f
};
constexpr UpdateRateFeedbackGains GAINS_MIN = {
    2.0f,
    0.1f
};
constexpr UpdateRateFeedbackGains GAINS_MAX = {
    8.0f,
    0.8f
};
constexpr UpdateRateFeedbackGains EXPERIMENT_FB_GAINS_INITIAL = GAIN_ZERO;
constexpr UpdateRateFeedbackGains EXPERIMENT_FB_GAINS_FINAL = GAIN_ZERO;

constexpr X0Vx0FeedbackGains EXPERIMENT_X0_VX0_FB_GAINS_INITIAL = {
    0.0047f,
    0.000008f
};
constexpr X0Vx0FeedbackGains EXPERIMENT_X0_VX0_FB_GAINS_FINAL = {
    0.0002f,
    0.00000008f
};

constexpr std::array<float, 3> TARGET_VELOCITY = {0.01f, 0.0f, 0.0f};
constexpr float VELOCITY_EPS = 1e-4f;

constexpr std::size_t EXPERIMENT_LOG_ROW_COUNT = 3000;

static_assert(
    EXPERIMENT_LOG_ROW_COUNT > EXPERIMENT_T_SUP_HOLD_STEP_COUNT,
    "The log must contain steps after the fixed T_sup interval");

#endif
