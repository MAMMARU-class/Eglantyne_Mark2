#ifndef EXPERIMENT_CONFIG_LOADER_H
#define EXPERIMENT_CONFIG_LOADER_H

#include <stddef.h>

#include "ExperimentConfig.h"

class ExperimentConfigLoader {
public:
    bool load(
        ExperimentConfig& destination,
        const char* options_path,
        const char* gain_path,
        const char* procedure_path);

    bool copy_loaded_files(
        const char* destination_prefix,
        const ExperimentConfig& config);

    const char* error_message() const { return error_buffer; }

private:
    bool load_options(ExperimentConfig& config, const char* path);
    bool load_gain_table(ExperimentConfig& config, const char* path);
    bool load_procedure(ExperimentConfig& config, const char* path);
    bool validate(const ExperimentConfig& config);
    void build_single_experiment(ExperimentConfig& config);
    bool copy_file(const char* source, const char* destination);
    bool write_generated_gain_file(
        const char* destination,
        const ExperimentConfig& config);
    bool write_generated_procedure_file(
        const char* destination,
        const ExperimentConfig& config);

    void set_error(const char* format, ...);

    char loaded_options_path[64] = {0};
    char loaded_gain_path[64] = {0};
    char loaded_procedure_path[64] = {0};
    char error_buffer[192] = {0};
};

#endif
