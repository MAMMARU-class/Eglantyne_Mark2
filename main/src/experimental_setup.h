#ifndef EXPERIMENTAL_SETUP_H
#define EXPERIMENTAL_SETUP_H

// options.ini is always loaded after the start switch is pressed. The gain
// and procedure CSV files are loaded only for non-single-T_sup experiments.
constexpr const char* EXPERIMENT_OPTIONS_PATH =
    "/config/options.ini";
constexpr const char* EXPERIMENT_UPDATE_RATE_GAIN_PATH =
    "/config/update_rate_gain.csv";
constexpr const char* EXPERIMENT_PROCEDURE_PATH =
    "/config/procedure.csv";

#endif
