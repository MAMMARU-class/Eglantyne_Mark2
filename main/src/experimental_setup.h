#ifndef EXPERIMENTAL_SETUP_H
#define EXPERIMENTAL_SETUP_H

// Experiment values are loaded from these files after the start switch is
// pressed. Invalid or missing files prevent the walking task from starting.
constexpr const char* EXPERIMENT_OPTIONS_PATH =
    "/config/options.ini";
constexpr const char* EXPERIMENT_UPDATE_RATE_GAIN_PATH =
    "/config/update_rate_gain.csv";
constexpr const char* EXPERIMENT_PROCEDURE_PATH =
    "/config/procedure.csv";

#endif
