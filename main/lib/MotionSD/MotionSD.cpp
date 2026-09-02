#include "MotionSD.h"

static Robot* robot;

MotionSD::MotionSD(){}

bool MotionSD::init(){
    SPI.begin(CLK, MISO, MOSI, CS);
    if (!SD.begin(CS)){
        Serial.println("Failed to initialize MotionSD card");
        return false;
    }

    // check Card type
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return false;
    }
    Serial.println("MotionSD card initialized");
    return true;
}

/* #########################################################################
WRITE and READ
##########################################################################*/
bool MotionSD::begin_csv_log(
    const char* filename,
    const char* const column_names[],
    size_t column_count,
    size_t row_capacity
){
    if (csv_log_buffer != nullptr) {
        free(csv_log_buffer);
        csv_log_buffer = nullptr;
    }

    csv_log_active = false;
    csv_log_filename = filename;
    csv_column_names = column_names;
    csv_column_count = 0;
    csv_log_row_capacity = 0;
    csv_log_row_count = 0;

    if (column_count == 0 || row_capacity == 0) {
        return false;
    }

    const size_t max_size = static_cast<size_t>(-1);
    if (row_capacity > max_size / column_count ||
        row_capacity * column_count > max_size / sizeof(float)) {
        Serial.println("CSV log buffer size overflow");
        return false;
    }

    const size_t allocation_size =
        sizeof(float) * column_count * row_capacity;
    csv_log_buffer = static_cast<float*>(
        malloc(allocation_size));
    if (csv_log_buffer == nullptr) {
        Serial.println("Failed to allocate CSV log buffer");
        Serial.print("Requested CSV log buffer bytes: ");
        Serial.println(allocation_size);
        return false;
    }

    csv_column_count = column_count;
    csv_log_row_capacity = row_capacity;
    csv_log_active = true;

    return true;
}

bool MotionSD::write_csv_row(
    const float values[],
    const bool valid[],
    size_t value_count
){
    if (!csv_log_active || csv_log_buffer == nullptr ||
        value_count != csv_column_count) {
        return false;
    }

    for (size_t i = 0; i < value_count; ++i) {
        size_t buffer_index = csv_log_row_count * csv_column_count + i;
        csv_log_buffer[buffer_index] = valid[i] ? values[i] : NAN;
    }

    csv_log_row_count++;
    if (csv_log_row_count >= csv_log_row_capacity) {
        csv_log_active = false;
        Serial.print("CSV capture buffer full: ");
        Serial.print(csv_log_row_count);
        Serial.println(" rows. Waiting for experiment end to save.");
    }

    return true;
}

bool MotionSD::write_csv_null_row(){
    if (!csv_log_active || csv_log_buffer == nullptr ||
        csv_column_count == 0) {
        return false;
    }

    for (size_t i = 0; i < csv_column_count; ++i) {
        size_t buffer_index = csv_log_row_count * csv_column_count + i;
        csv_log_buffer[buffer_index] = NAN;
    }

    csv_log_row_count++;
    if (csv_log_row_count >= csv_log_row_capacity) {
        csv_log_active = false;
        Serial.print("CSV capture buffer full: ");
        Serial.print(csv_log_row_count);
        Serial.println(" rows. Waiting for experiment end to save.");
    }

    return true;
}

bool MotionSD::finish_csv_log(){
    if (csv_log_buffer == nullptr || csv_column_count == 0) {
        return false;
    }

    csv_log_active = false;
    File file = SD.open(csv_log_filename.c_str(), FILE_WRITE);
    if (!file) {
        Serial.print("Failed to open CSV log: ");
        Serial.println(csv_log_filename.c_str());
        return false;
    }

    for (size_t i = 0; i < csv_column_count; ++i) {
        file.print(csv_column_names[i]);
        if (i + 1 < csv_column_count) {
            file.print(",");
        }
    }
    file.println();

    for (size_t row = 0; row < csv_log_row_count; ++row) {
        for (size_t column = 0; column < csv_column_count; ++column) {
            float value = csv_log_buffer[row * csv_column_count + column];
            if (isnan(value)) {
                file.print("Null");
            } else {
                file.print(value, 6);
            }
            if (column + 1 < csv_column_count) {
                file.print(",");
            }
        }
        file.println();
    }

    file.close();
    free(csv_log_buffer);
    csv_log_buffer = nullptr;

    Serial.print("CSV capture completed: ");
    Serial.print(csv_log_row_count);
    Serial.println(" rows");
    return true;
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

void MotionSD::create_directory(const char* dirname){
    if (!SD.exists(dirname)) {
        SD.mkdir(dirname);
    }
}
