#ifndef MOTIONSD_H
#define MOTIONSD_H

#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "CubicSpline.h"

#include "Robot.h"

#define CS 10
#define MOSI 11
#define CLK 12
#define MISO 13

using std::array;

class MotionSD{
public:
    MotionSD();

    void init();

    bool begin_csv_log(
        const char* filename,
        const char* const column_names[],
        size_t column_count,
        size_t row_capacity);
    bool write_csv_row(
        const float values[],
        const bool valid[],
        size_t value_count);
    bool write_csv_null_row();
    bool finish_csv_log();
    bool is_csv_log_active() const { return csv_log_active; }
    size_t get_csv_log_row_count() const { return csv_log_row_count; }
    
    array<float, 18> read_motion(
        const char* filename,
        size_t id);

    void play_motion(
        Robot* r,
        const char* fname,
        float duration);

    std::string get_filename_by_id(size_t id);
    
    void delete_motion_file(const char* filename);

    bool is_file_exist(const char* filename);
    void create_directory(const char* dirname);

private:
    std::string csv_log_filename;
    const char* const* csv_column_names = nullptr;
    float* csv_log_buffer = nullptr;
    size_t csv_column_count = 0;
    size_t csv_log_row_capacity = 0;
    size_t csv_log_row_count = 0;
    bool csv_log_active = false;
};

#endif
