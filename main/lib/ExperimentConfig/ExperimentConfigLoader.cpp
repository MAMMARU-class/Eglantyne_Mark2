#include "ExperimentConfigLoader.h"

#include <Arduino.h>
#include <SD.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr size_t CONFIG_LINE_BUFFER_SIZE = 192;
constexpr float MIN_T_SUP = 0.05f;
constexpr float MAX_T_SUP = 1.0f;
constexpr float MAX_FEEDBACK_GAIN = 1000.0f;
constexpr float MAX_TARGET_VELOCITY = 1.0f;
constexpr size_t MAX_STEPS_PER_PROCEDURE_ITEM = 10000;
constexpr float T_SUP_CONTINUITY_TOLERANCE = 1.0e-5f;

char* trim(char* text){
    while (*text != '\0' && isspace(static_cast<unsigned char>(*text))) {
        ++text;
    }

    char* end = text + strlen(text);
    while (end > text &&
           isspace(static_cast<unsigned char>(*(end - 1)))) {
        --end;
    }
    *end = '\0';
    return text;
}

bool equals_ignore_case(const char* left, const char* right){
    while (*left != '\0' && *right != '\0') {
        if (toupper(static_cast<unsigned char>(*left)) !=
            toupper(static_cast<unsigned char>(*right))) {
            return false;
        }
        ++left;
        ++right;
    }
    return *left == '\0' && *right == '\0';
}

bool parse_float_value(const char* text, float& value){
    errno = 0;
    char* end = nullptr;
    value = strtof(text, &end);
    return end != text && *trim(end) == '\0' && errno != ERANGE &&
        isfinite(value);
}

bool parse_size_value(const char* text, size_t& value){
    if (*text == '-') {
        return false;
    }

    errno = 0;
    char* end = nullptr;
    const unsigned long parsed = strtoul(text, &end, 10);
    if (end == text || *trim(end) != '\0' || errno == ERANGE) {
        return false;
    }
    value = static_cast<size_t>(parsed);
    return static_cast<unsigned long>(value) == parsed;
}

bool parse_disturbance_type(
    const char* text,
    DisturbanceType& type)
{
    if (equals_ignore_case(text, "NONE")) {
        type = DisturbanceType::NONE;
        return true;
    }
    if (equals_ignore_case(text, "PUSH")) {
        type = DisturbanceType::PUSH;
        return true;
    }
    if (equals_ignore_case(text, "STEP")) {
        type = DisturbanceType::STEP;
        return true;
    }
    return false;
}

bool parse_feedback_gain_mode(
    const char* text,
    FeedbackGainMode& mode)
{
    if (equals_ignore_case(text, "FIXED_MAXIMUM")) {
        mode = FeedbackGainMode::FIXED_MAXIMUM;
        return true;
    }
    if (equals_ignore_case(text, "T_SUP_DEPENDENT")) {
        mode = FeedbackGainMode::T_SUP_DEPENDENT;
        return true;
    }
    if (equals_ignore_case(text, "FIXED_MINIMUM")) {
        mode = FeedbackGainMode::FIXED_MINIMUM;
        return true;
    }
    return false;
}

bool parse_error_action(const char* text, ErrorAction& action){
    if (equals_ignore_case(text, "RESTART")) {
        action = ErrorAction::RESTART;
        return true;
    }
    if (equals_ignore_case(text, "SKIP")) {
        action = ErrorAction::SKIP;
        return true;
    }
    return false;
}

bool parse_experiment_content(
    const char* text,
    ExperimentContent& content)
{
    if (equals_ignore_case(text, "WARMUP")) {
        content = ExperimentContent::WARMUP;
        return true;
    }
    if (equals_ignore_case(text, "WALK")) {
        content = ExperimentContent::WALK;
        return true;
    }
    if (equals_ignore_case(text, "ENDING")) {
        content = ExperimentContent::ENDING;
        return true;
    }
    return false;
}

size_t split_csv(char* line, char* fields[], size_t max_fields){
    size_t count = 0;
    char* field_start = line;

    while (true) {
        if (count >= max_fields) {
            return max_fields + 1;
        }
        fields[count++] = field_start;

        char* comma = strchr(field_start, ',');
        if (comma == nullptr) {
            break;
        }
        *comma = '\0';
        field_start = comma + 1;
    }

    for (size_t index = 0; index < count; ++index) {
        fields[index] = trim(fields[index]);
    }
    return count;
}

bool read_config_line(
    File& file,
    char* destination,
    size_t destination_size,
    bool& too_long)
{
    if (!file.available()) {
        return false;
    }

    String line = file.readStringUntil('\n');
    if (line.endsWith("\r")) {
        line.remove(line.length() - 1);
    }
    too_long = line.length() >= destination_size;
    if (too_long) {
        destination[0] = '\0';
        return true;
    }
    line.toCharArray(destination, destination_size);
    return true;
}

void remove_comment(char* line){
    char* comment = strchr(line, '#');
    if (comment != nullptr) {
        *comment = '\0';
    }
}

void remove_utf8_bom(char* line){
    const unsigned char* bytes =
        reinterpret_cast<const unsigned char*>(line);
    if (strlen(line) >= 3 &&
        bytes[0] == 0xEF && bytes[1] == 0xBB && bytes[2] == 0xBF) {
        memmove(line, line + 3, strlen(line + 3) + 1);
    }
}

}  // namespace

bool ExperimentConfigLoader::load(
    ExperimentConfig& destination,
    const char* options_path,
    const char* gain_path,
    const char* procedure_path)
{
    error_buffer[0] = '\0';
    ExperimentConfig candidate;

    if (!load_options(candidate, options_path) ||
        !load_gain_table(candidate, gain_path) ||
        !load_procedure(candidate, procedure_path) ||
        !validate(candidate)) {
        return false;
    }

    destination = candidate;
    snprintf(
        loaded_options_path,
        sizeof(loaded_options_path),
        "%s",
        options_path);
    snprintf(
        loaded_gain_path,
        sizeof(loaded_gain_path),
        "%s",
        gain_path);
    snprintf(
        loaded_procedure_path,
        sizeof(loaded_procedure_path),
        "%s",
        procedure_path);
    return true;
}

bool ExperimentConfigLoader::load_options(
    ExperimentConfig& config,
    const char* path)
{
    File file = SD.open(path, FILE_READ);
    if (!file) {
        set_error("Cannot open options file: %s", path);
        return false;
    }

    enum OptionKey : uint8_t {
        VERSION,
        FEEDBACK_MODE,
        DISTURBANCE_TYPE,
        X0_KP,
        X0_KD,
        VELOCITY_X,
        VELOCITY_Y,
        VELOCITY_YAW,
        VELOCITY_EPSILON,
        LOG_ROWS,
        OPTION_KEY_COUNT
    };

    uint16_t seen_keys = 0;
    size_t line_number = 0;
    char line[CONFIG_LINE_BUFFER_SIZE];
    bool too_long = false;

    while (read_config_line(
        file, line, sizeof(line), too_long)) {
        ++line_number;
        if (too_long) {
            file.close();
            set_error("%s:%u line is too long", path,
                      static_cast<unsigned>(line_number));
            return false;
        }

        remove_utf8_bom(line);
        remove_comment(line);
        char* content = trim(line);
        if (*content == '\0') {
            continue;
        }

        char* equals = strchr(content, '=');
        if (equals == nullptr || strchr(equals + 1, '=') != nullptr) {
            file.close();
            set_error("%s:%u expected key=value", path,
                      static_cast<unsigned>(line_number));
            return false;
        }
        *equals = '\0';
        char* key = trim(content);
        char* value = trim(equals + 1);
        if (*key == '\0' || *value == '\0') {
            file.close();
            set_error("%s:%u key or value is empty", path,
                      static_cast<unsigned>(line_number));
            return false;
        }

        int key_id = -1;
        if (equals_ignore_case(key, "version")) {
            key_id = VERSION;
        }else if (equals_ignore_case(key, "feedback_gain_mode")) {
            key_id = FEEDBACK_MODE;
        }else if (equals_ignore_case(key, "disturbance_type")) {
            key_id = DISTURBANCE_TYPE;
        }else if (equals_ignore_case(key, "x0_vx0_kp")) {
            key_id = X0_KP;
        }else if (equals_ignore_case(key, "x0_vx0_kd")) {
            key_id = X0_KD;
        }else if (equals_ignore_case(key, "target_velocity_x")) {
            key_id = VELOCITY_X;
        }else if (equals_ignore_case(key, "target_velocity_y")) {
            key_id = VELOCITY_Y;
        }else if (equals_ignore_case(key, "target_velocity_yaw")) {
            key_id = VELOCITY_YAW;
        }else if (equals_ignore_case(key, "velocity_eps")) {
            key_id = VELOCITY_EPSILON;
        }else if (equals_ignore_case(key, "log_row_count")) {
            key_id = LOG_ROWS;
        }

        if (key_id < 0) {
            file.close();
            set_error("%s:%u unknown option: %s", path,
                      static_cast<unsigned>(line_number), key);
            return false;
        }

        const uint16_t key_mask =
            static_cast<uint16_t>(1U << key_id);
        if ((seen_keys & key_mask) != 0) {
            file.close();
            set_error("%s:%u duplicate option: %s", path,
                      static_cast<unsigned>(line_number), key);
            return false;
        }

        bool parsed = false;
        switch (key_id) {
            case VERSION: {
                size_t version = 0;
                parsed = parse_size_value(value, version) && version == 1;
                break;
            }
            case FEEDBACK_MODE:
                parsed = parse_feedback_gain_mode(
                    value, config.feedback_gain_mode);
                break;
            case DISTURBANCE_TYPE:
                parsed = parse_disturbance_type(
                    value, config.disturbance_type);
                break;
            case X0_KP:
                parsed = parse_float_value(
                    value, config.x0_vx0_gains.kp);
                break;
            case X0_KD:
                parsed = parse_float_value(
                    value, config.x0_vx0_gains.kd);
                break;
            case VELOCITY_X:
                parsed = parse_float_value(
                    value, config.target_velocity[0]);
                break;
            case VELOCITY_Y:
                parsed = parse_float_value(
                    value, config.target_velocity[1]);
                break;
            case VELOCITY_YAW:
                parsed = parse_float_value(
                    value, config.target_velocity[2]);
                break;
            case VELOCITY_EPSILON:
                parsed = parse_float_value(value, config.velocity_eps);
                break;
            case LOG_ROWS:
                parsed = parse_size_value(value, config.log_row_count);
                break;
            default:
                break;
        }

        if (!parsed) {
            file.close();
            set_error("%s:%u invalid value for %s", path,
                      static_cast<unsigned>(line_number), key);
            return false;
        }
        seen_keys |= key_mask;
    }

    file.close();
    const uint16_t all_keys =
        static_cast<uint16_t>((1U << OPTION_KEY_COUNT) - 1U);
    if (seen_keys != all_keys) {
        set_error("Options file is missing one or more required values: %s",
                  path);
        return false;
    }
    return true;
}

bool ExperimentConfigLoader::load_gain_table(
    ExperimentConfig& config,
    const char* path)
{
    File file = SD.open(path, FILE_READ);
    if (!file) {
        set_error("Cannot open update-rate gain file: %s", path);
        return false;
    }

    bool header_read = false;
    size_t line_number = 0;
    char line[CONFIG_LINE_BUFFER_SIZE];
    bool too_long = false;

    while (read_config_line(
        file, line, sizeof(line), too_long)) {
        ++line_number;
        if (too_long) {
            file.close();
            set_error("%s:%u line is too long", path,
                      static_cast<unsigned>(line_number));
            return false;
        }

        remove_utf8_bom(line);
        remove_comment(line);
        char* content = trim(line);
        if (*content == '\0') {
            continue;
        }

        char* fields[3];
        const size_t field_count = split_csv(content, fields, 3);
        if (field_count != 3) {
            file.close();
            set_error("%s:%u expected 3 columns", path,
                      static_cast<unsigned>(line_number));
            return false;
        }

        if (!header_read) {
            if (!equals_ignore_case(fields[0], "t_sup") ||
                !equals_ignore_case(fields[1], "kp") ||
                !equals_ignore_case(fields[2], "kd")) {
                file.close();
                set_error("%s:%u invalid CSV header", path,
                          static_cast<unsigned>(line_number));
                return false;
            }
            header_read = true;
            continue;
        }

        if (config.update_rate_gain_count >=
            EXPERIMENT_CONFIG_MAX_GAIN_POINTS) {
            file.close();
            set_error("%s has more than %u gain points", path,
                      static_cast<unsigned>(
                          EXPERIMENT_CONFIG_MAX_GAIN_POINTS));
            return false;
        }

        UpdateRateGainPoint point;
        if (!parse_float_value(fields[0], point.t_sup) ||
            !parse_float_value(fields[1], point.kp) ||
            !parse_float_value(fields[2], point.kd)) {
            file.close();
            set_error("%s:%u invalid numeric gain value", path,
                      static_cast<unsigned>(line_number));
            return false;
        }

        if (config.update_rate_gain_count > 0 &&
            point.t_sup <= config.update_rate_gain_table[
                config.update_rate_gain_count - 1].t_sup) {
            file.close();
            set_error("%s:%u t_sup values must be strictly increasing",
                      path, static_cast<unsigned>(line_number));
            return false;
        }

        config.update_rate_gain_table[
            config.update_rate_gain_count++] = point;
    }

    file.close();
    if (!header_read || config.update_rate_gain_count == 0) {
        set_error("Update-rate gain table is empty: %s", path);
        return false;
    }
    return true;
}

bool ExperimentConfigLoader::load_procedure(
    ExperimentConfig& config,
    const char* path)
{
    File file = SD.open(path, FILE_READ);
    if (!file) {
        set_error("Cannot open procedure file: %s", path);
        return false;
    }

    bool header_read = false;
    size_t line_number = 0;
    char line[CONFIG_LINE_BUFFER_SIZE];
    bool too_long = false;

    while (read_config_line(
        file, line, sizeof(line), too_long)) {
        ++line_number;
        if (too_long) {
            file.close();
            set_error("%s:%u line is too long", path,
                      static_cast<unsigned>(line_number));
            return false;
        }

        remove_utf8_bom(line);
        remove_comment(line);
        char* content = trim(line);
        if (*content == '\0') {
            continue;
        }

        char* fields[5];
        const size_t field_count = split_csv(content, fields, 5);
        if (field_count != 5) {
            file.close();
            set_error("%s:%u expected 5 columns", path,
                      static_cast<unsigned>(line_number));
            return false;
        }

        if (!header_read) {
            if (!equals_ignore_case(fields[0], "t_sup_start") ||
                !equals_ignore_case(fields[1], "t_sup_end") ||
                !equals_ignore_case(fields[2], "steps") ||
                !equals_ignore_case(fields[3], "error_action") ||
                !equals_ignore_case(fields[4], "content")) {
                file.close();
                set_error("%s:%u invalid CSV header", path,
                          static_cast<unsigned>(line_number));
                return false;
            }
            header_read = true;
            continue;
        }

        if (config.procedure_count >=
            EXPERIMENT_CONFIG_MAX_PROCEDURE_ITEMS) {
            file.close();
            set_error("%s has more than %u procedure items", path,
                      static_cast<unsigned>(
                          EXPERIMENT_CONFIG_MAX_PROCEDURE_ITEMS));
            return false;
        }

        ExperimentProcedureItem item;
        if (!parse_float_value(fields[0], item.t_sup_start) ||
            !parse_float_value(fields[1], item.t_sup_end) ||
            !parse_size_value(fields[2], item.step_count) ||
            !parse_error_action(fields[3], item.error_action) ||
            !parse_experiment_content(fields[4], item.content)) {
            file.close();
            set_error("%s:%u invalid procedure value", path,
                      static_cast<unsigned>(line_number));
            return false;
        }

        config.procedure[config.procedure_count++] = item;
    }

    file.close();
    if (!header_read || config.procedure_count == 0) {
        set_error("Experiment procedure is empty: %s", path);
        return false;
    }
    return true;
}

bool ExperimentConfigLoader::validate(const ExperimentConfig& config){
    if (config.log_row_count == 0) {
        set_error("log_row_count must be greater than zero");
        return false;
    }

    if (config.x0_vx0_gains.kp < 0.0f ||
        config.x0_vx0_gains.kd < 0.0f ||
        config.x0_vx0_gains.kp > MAX_FEEDBACK_GAIN ||
        config.x0_vx0_gains.kd > MAX_FEEDBACK_GAIN) {
        set_error("x0/vx0 gains are outside the safe range");
        return false;
    }

    for (size_t axis = 0; axis < config.target_velocity.size(); ++axis) {
        if (fabsf(config.target_velocity[axis]) > MAX_TARGET_VELOCITY) {
            set_error("target velocity is outside the safe range");
            return false;
        }
    }
    if (config.velocity_eps <= 0.0f || config.velocity_eps > 1.0f) {
        set_error("velocity_eps must be in the range (0, 1]");
        return false;
    }

    if (config.feedback_gain_mode == FeedbackGainMode::T_SUP_DEPENDENT &&
        config.update_rate_gain_count < 2) {
        set_error("T_SUP_DEPENDENT requires at least two gain points");
        return false;
    }

    for (size_t index = 0;
         index < config.update_rate_gain_count;
         ++index) {
        const UpdateRateGainPoint& point =
            config.update_rate_gain_table[index];
        if (point.t_sup < MIN_T_SUP || point.t_sup > MAX_T_SUP ||
            point.kp < 0.0f || point.kd < 0.0f ||
            point.kp > MAX_FEEDBACK_GAIN ||
            point.kd > MAX_FEEDBACK_GAIN) {
            set_error("Update-rate gain point %u is outside the safe range",
                      static_cast<unsigned>(index));
            return false;
        }
    }

    const float gain_t_sup_min = config.update_rate_gain_table[0].t_sup;
    const float gain_t_sup_max = config.update_rate_gain_table[
        config.update_rate_gain_count - 1].t_sup;
    for (size_t index = 0; index < config.procedure_count; ++index) {
        const ExperimentProcedureItem& item = config.procedure[index];
        if (item.t_sup_start < MIN_T_SUP ||
            item.t_sup_start > MAX_T_SUP ||
            item.t_sup_end < MIN_T_SUP ||
            item.t_sup_end > MAX_T_SUP) {
            set_error("Procedure item %u has an unsafe t_sup",
                      static_cast<unsigned>(index));
            return false;
        }
        if (item.step_count == 0 ||
            item.step_count > MAX_STEPS_PER_PROCEDURE_ITEM) {
            set_error("Procedure item %u has an invalid step count",
                      static_cast<unsigned>(index));
            return false;
        }
        const bool t_sup_changes =
            fabsf(item.t_sup_end - item.t_sup_start) >
                T_SUP_CONTINUITY_TOLERANCE;
        if (t_sup_changes && item.content != ExperimentContent::WARMUP) {
            set_error(
                "Procedure item %u changes t_sup outside WARMUP",
                static_cast<unsigned>(index));
            return false;
        }
        if (t_sup_changes && item.step_count < 2) {
            set_error(
                "Procedure item %u needs at least 2 steps to change t_sup",
                static_cast<unsigned>(index));
            return false;
        }
        if (index > 0 &&
            fabsf(config.procedure[index - 1].t_sup_end -
                  item.t_sup_start) > T_SUP_CONTINUITY_TOLERANCE) {
            set_error(
                "Procedure item %u is discontinuous with the previous item",
                static_cast<unsigned>(index));
            return false;
        }
        if (config.feedback_gain_mode ==
                FeedbackGainMode::T_SUP_DEPENDENT &&
            (item.t_sup_start < gain_t_sup_min ||
             item.t_sup_start > gain_t_sup_max ||
             item.t_sup_end < gain_t_sup_min ||
             item.t_sup_end > gain_t_sup_max)) {
            set_error(
                "Procedure item %u t_sup is outside the gain table range",
                static_cast<unsigned>(index));
            return false;
        }
    }

    return true;
}

bool ExperimentConfigLoader::copy_loaded_files(
    const char* destination_prefix)
{
    if (loaded_options_path[0] == '\0' ||
        loaded_gain_path[0] == '\0' ||
        loaded_procedure_path[0] == '\0') {
        set_error("No successfully loaded configuration to copy");
        return false;
    }

    char destination[128];
    snprintf(destination, sizeof(destination), "%s_options.ini",
             destination_prefix);
    if (!copy_file(loaded_options_path, destination)) {
        return false;
    }

    snprintf(destination, sizeof(destination), "%s_gain.csv",
             destination_prefix);
    if (!copy_file(loaded_gain_path, destination)) {
        return false;
    }

    snprintf(destination, sizeof(destination), "%s_procedure.csv",
             destination_prefix);
    return copy_file(loaded_procedure_path, destination);
}

bool ExperimentConfigLoader::copy_file(
    const char* source,
    const char* destination)
{
    File input = SD.open(source, FILE_READ);
    if (!input) {
        set_error("Cannot reopen configuration file: %s", source);
        return false;
    }

    if (SD.exists(destination) && !SD.remove(destination)) {
        input.close();
        set_error("Cannot replace configuration copy: %s", destination);
        return false;
    }

    File output = SD.open(destination, FILE_WRITE);
    if (!output) {
        input.close();
        set_error("Cannot create configuration copy: %s", destination);
        return false;
    }

    uint8_t buffer[256];
    while (input.available()) {
        const size_t bytes_read = input.read(buffer, sizeof(buffer));
        if (bytes_read == 0 || output.write(buffer, bytes_read) != bytes_read) {
            input.close();
            output.close();
            set_error("Failed while copying configuration to: %s",
                      destination);
            return false;
        }
    }

    input.close();
    output.close();
    return true;
}

void ExperimentConfigLoader::set_error(const char* format, ...){
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(error_buffer, sizeof(error_buffer), format, arguments);
    va_end(arguments);
}
