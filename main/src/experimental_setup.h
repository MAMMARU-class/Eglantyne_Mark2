#ifndef EXPERIMENTAL_SETUP_H
#define EXPERIMENTAL_SETUP_H

#include <array>

constexpr float EXPERIMENT_T_SUP = 0.14f;
constexpr bool EXPERIMENT_FB_ENABLED = true;
constexpr bool EXPERIMENT_DISTURBANCE_ENABLED = true;
constexpr std::array<float, 3> TARGET_VELOCITY = {0.1f, 0.0f, 0.0f};
constexpr float VELOCITY_EPS = 1e-4f;

constexpr std::size_t EXPERIMENT_LOG_ROW_COUNT = 1200;

#endif
