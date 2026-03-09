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

    void write_motion(
        const char* filename,
        array<float, 18>);
    
    void write_long_motion(
        const char* filename,
        float motions[][18],
        int length
    );
    
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

private:

};

#endif