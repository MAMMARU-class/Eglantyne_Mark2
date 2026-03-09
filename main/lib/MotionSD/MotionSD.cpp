#include "MotionSD.h"

static Robot* robot;

MotionSD::MotionSD(){}

void MotionSD::init(){
    SPI.begin(CLK, MISO, MOSI, CS);
    if (!SD.begin(CS)){
        Serial.println("Failed to initialize MotionSD card");
        delay(100);
    }

    // check Card type
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return;
    }
    Serial.println("MotionSD card initialized");
}

/* #########################################################################
WRITE and READ
##########################################################################*/
void MotionSD::write_motion(
    const char* filename,
    array<float, 18> data
){
    Serial.print("Writing: ");
    for (const auto &v : data) {
        Serial.print(v, 3); Serial.print(" ");
    }
    Serial.println();

    File file = SD.open(filename, FILE_APPEND);
    if (!file) return;

    for (size_t i = 0; i < 18; ++i) {
        file.print(data[i], 6);
        if (i < 17) file.print(",");
    }
    file.println();

    file.close();
}

void MotionSD::write_long_motion(
    const char* filename,
    float motions[][18],
    int length
){
    File file = SD.open(filename, FILE_APPEND);
    if (!file) return;

    for (int k = 0; k < length; k++) {
        for (int i = 0; i < 18; i++) {
            file.print(motions[k][i], 6);
            if (i < 17) file.print(",");
        }
        file.println();
    }

    file.close();
}

array<float, 18> MotionSD::read_motion(
    const char* filename,
    size_t id)
{
    std::array<float, 18> result;

    // デフォルトは NaN
    for (auto &v : result) {
        v = NAN;
    }

    File file = SD.open(filename, FILE_READ);
    if (!file) return result;

    size_t current_line = 0;
    char buffer[256];

    while (file.available()) {

        size_t len = file.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        buffer[len] = '\0';

        if (current_line == id) {

            char* token = strtok(buffer, ",");
            size_t index = 0;

            while (token != nullptr && index < 18) {
                result[index++] = atof(token);
                token = strtok(nullptr, ",");
            }

            if (index != 18) {
                for (auto &v : result) v = NAN;
            }

            file.close();
            return result;
        }

        current_line++;
    }

    file.close();

    // Serial.print("data: ");
    // for (const auto &v : result) {
    //     Serial.print(v, 3); Serial.print(" ");
    // }
    Serial.println();
    return result;
}

/* #########################################################################
PLAY
##########################################################################*/
void MotionSD::play_motion(Robot* r, const char* fname, float duration){
    robot = r;
    size_t id = 0;
    std::vector<array<float, LINK_SIZE>> motions;
    motions.reserve(100);

    while(1){
        array<float, LINK_SIZE> motion = this->read_motion(fname, id);
        if (std::isnan(motion[0])){
            break;
        }
        motions.push_back(motion);
        id++;
    }

    if (motions.empty()) {
        Serial.println("No motion data found.");
        return;
    }
    if(motions.size() < 2){
        robot->move_all_t(motions[0], duration);
        return;
    }

    // move to first position
    robot->move_all_t(motions[0], 0.5);

    size_t N = motions.size();
    // create time vector
    std::vector<double> t(N);
    for(size_t i=0;i<N;i++){
        t[i] = i * duration;
    }

    // create splines for each joint
    std::vector<CubicSpline1D> splines(LINK_SIZE);
    for(size_t joint=0; joint<LINK_SIZE; joint++){
        std::vector<double> y(N);
        for(size_t i=0;i<N;i++){
            y[i] = motions[i][joint];
        }
        splines[joint].build(t, y);
    }

    // play motion
    double total_time = (N-1) * duration;

    double dt = 1.0/100.0; // 100Hz
    for(double tt=0; tt<=total_time; tt+=dt){
        array<float, LINK_SIZE> target;

        for(size_t joint=0; joint<LINK_SIZE; joint++){
            target[joint] = 
                static_cast<float>(splines[joint].eval(tt));
        }
        robot->move_all(target);

        delay(dt*1000);
    }
}

/* #########################################################################
FILE NAME
##########################################################################*/
std::string MotionSD::get_filename_by_id(size_t id){
    File root = SD.open("/");
    if (!root) return "";

    size_t count = 0;

    // まずファイル数を数える
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            count++;
        }
        file = root.openNextFile();
    }

    if (count == 0) {
        root.close();
        return "";
    }

    size_t target = id % count;

    // 先頭に戻す
    root.rewindDirectory();

    size_t index = 0;
    file = root.openNextFile();

    while (file) {

        if (!file.isDirectory()) {

            if (index == target) {
                std::string name = file.name();
                file.close();
                root.close();
                return name;
            }

            index++;
        }

        file = root.openNextFile();
    }

    root.close();
    return "";
}

void MotionSD::delete_motion_file(const char* filename){
    if (!SD.exists(filename)) {
        Serial.print("File not found: ");
        Serial.println(filename);
        return;
    }

    // create empty file to overwrite
    File file = SD.open(filename, FILE_WRITE);
    file.close();
}

bool MotionSD::is_file_exist(const char* filename){
    return SD.exists(filename);
}
