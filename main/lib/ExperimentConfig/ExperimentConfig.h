#ifndef EXPERIMENT_CONFIG_H
#define EXPERIMENT_CONFIG_H

#include <array>
#include <stddef.h>
#include <stdint.h>

#include "ExperimentManager.h"

enum class FeedbackGainMode : uint8_t {
    FIXED_MAXIMUM,
    T_SUP_DEPENDENT,
    FIXED_MINIMUM
};

enum class DisturbanceType : uint8_t {
    NONE,
    PUSH,
    STEP
};

struct UpdateRateFeedbackGains {
    float kp;
    float kd;
};

struct X0Vx0FeedbackGains {
    float kp;
    float kd;
};

struct PitchFootFeedbackGains {
    float kp;
    float kd;
};

struct UpdateRateGainPoint {
    float t_sup;
    float kp;
    float kd;
};

constexpr size_t EXPERIMENT_CONFIG_MAX_GAIN_POINTS = 32;
constexpr size_t EXPERIMENT_CONFIG_MAX_PROCEDURE_ITEMS = 64;

struct ExperimentConfig {
    FeedbackGainMode feedback_gain_mode =
        FeedbackGainMode::T_SUP_DEPENDENT;
    DisturbanceType disturbance_type = DisturbanceType::NONE;
    bool single_t_sup_exp = false;
    float single_t_sup = 0.0f;
    float single_gain_p = 0.0f;
    float single_gain_d = 0.0f;
    X0Vx0FeedbackGains x0_vx0_gains = {0.0f, 0.0f};
    PitchFootFeedbackGains pitch_foot_gains = {0.0f, 0.0f};
    std::array<float, 3> target_velocity = {{0.0f, 0.0f, 0.0f}};
    float velocity_eps = 0.0f;
    size_t log_row_count = 0;

    std::array<UpdateRateGainPoint,
               EXPERIMENT_CONFIG_MAX_GAIN_POINTS> update_rate_gain_table;
    size_t update_rate_gain_count = 0;

    std::array<ExperimentProcedureItem,
               EXPERIMENT_CONFIG_MAX_PROCEDURE_ITEMS> procedure;
    size_t procedure_count = 0;

    UpdateRateFeedbackGains calculate_update_rate_gains(float t_sup) const;
};

const char* feedback_gain_mode_name(FeedbackGainMode mode);
const char* disturbance_type_name(DisturbanceType type);
const char* experiment_content_name(ExperimentContent content);
const char* error_action_name(ErrorAction action);

#endif
