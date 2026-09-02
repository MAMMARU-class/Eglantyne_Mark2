#include "ExperimentConfig.h"

UpdateRateFeedbackGains ExperimentConfig::calculate_update_rate_gains(
    float t_sup) const
{
    if (update_rate_gain_count == 0) {
        return {0.0f, 0.0f};
    }

    if (feedback_gain_mode == FeedbackGainMode::FIXED_MINIMUM) {
        return {
            update_rate_gain_table[0].kp,
            update_rate_gain_table[0].kd
        };
    }

    const size_t last_index = update_rate_gain_count - 1;
    if (feedback_gain_mode == FeedbackGainMode::FIXED_MAXIMUM) {
        return {
            update_rate_gain_table[last_index].kp,
            update_rate_gain_table[last_index].kd
        };
    }

    if (t_sup <= update_rate_gain_table[0].t_sup) {
        return {
            update_rate_gain_table[0].kp,
            update_rate_gain_table[0].kd
        };
    }
    if (t_sup >= update_rate_gain_table[last_index].t_sup) {
        return {
            update_rate_gain_table[last_index].kp,
            update_rate_gain_table[last_index].kd
        };
    }

    for (size_t upper_index = 1;
         upper_index < update_rate_gain_count;
         ++upper_index) {
        const UpdateRateGainPoint& upper =
            update_rate_gain_table[upper_index];
        if (t_sup <= upper.t_sup) {
            const UpdateRateGainPoint& lower =
                update_rate_gain_table[upper_index - 1];
            const float ratio =
                (t_sup - lower.t_sup) / (upper.t_sup - lower.t_sup);
            return {
                lower.kp + (upper.kp - lower.kp) * ratio,
                lower.kd + (upper.kd - lower.kd) * ratio
            };
        }
    }

    return {
        update_rate_gain_table[last_index].kp,
        update_rate_gain_table[last_index].kd
    };
}

const char* feedback_gain_mode_name(FeedbackGainMode mode){
    switch (mode){
        case FeedbackGainMode::FIXED_MAXIMUM:
            return "fixed_max";
        case FeedbackGainMode::T_SUP_DEPENDENT:
            return "t_sup_function";
        case FeedbackGainMode::FIXED_MINIMUM:
            return "fixed_min";
    }
    return "unknown";
}

const char* disturbance_type_name(DisturbanceType type){
    switch (type){
        case DisturbanceType::NONE:
            return "NONE";
        case DisturbanceType::PUSH:
            return "PUSH";
        case DisturbanceType::STEP:
            return "STEP";
    }
    return "UNKNOWN";
}

const char* experiment_content_name(ExperimentContent content){
    switch (content){
        case ExperimentContent::WARMUP:
            return "WARMUP";
        case ExperimentContent::WALK:
            return "WALK";
        case ExperimentContent::ENDING:
            return "ENDING";
    }
    return "UNKNOWN";
}

const char* error_action_name(ErrorAction action){
    switch (action){
        case ErrorAction::RESTART:
            return "RESTART";
        case ErrorAction::SKIP:
            return "SKIP";
    }
    return "UNKNOWN";
}
