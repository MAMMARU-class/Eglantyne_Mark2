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
READ and WRITE
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

    Serial.print("data: ");
    for (const auto &v : result) {
        Serial.print(v, 3); Serial.print(" ");
    }
    Serial.println();
    return result;
}

void MotionSD::play_motion(Robot* r, const char* fname, float duration){
    robot = r;
    size_t id = 0;
    while(1){
        array<float, LINK_SIZE> motion = read_motion(fname, id);
        if (std::isnan(motion[0])){
            break;
        }
        robot->move_all_t(motion, duration);
        id++;
    }
}

/* #########################################################################
INSERT and DELETE
##########################################################################*/
void MotionSD::insert_motion(
    const char* filename,
    size_t id,
    std::array<float,18> data
){
    File src = SD.open(filename, FILE_READ);
    if (!src) return;

    File tmp = SD.open("/tmp.csv", FILE_WRITE);
    if (!tmp) {
        src.close();
        return;
    }

    size_t current_line = 0;
    bool inserted = false;
    char buffer[256];

    while (src.available()) {

        size_t len = src.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        buffer[len] = '\0';

        if (current_line == id && !inserted) {

            // --- 新データ書き込み ---
            for (size_t i = 0; i < 18; ++i) {
                tmp.print(data[i], 6);
                if (i < 17) tmp.print(",");
            }
            tmp.println();

            inserted = true;
        }

        // --- 元行を書き込み ---
        tmp.println(buffer);

        current_line++;
    }

    // id が最終行より大きい場合
    if (!inserted) {
        for (size_t i = 0; i < 18; ++i) {
            tmp.print(data[i], 6);
            if (i < 17) tmp.print(",");
        }
        tmp.println();
    }

    src.close();
    tmp.close();

    SD.remove(filename);
    SD.rename("/tmp.csv", filename);
}

void MotionSD::delete_motion_line(
    const char* filename,
    size_t id
){
    File src = SD.open(filename, FILE_READ);
    if (!src) return;

    File tmp = SD.open("/tmp.csv", FILE_WRITE);
    if (!tmp) {
        src.close();
        return;
    }

    size_t current_line = 0;
    char buffer[256];

    while (src.available()) {

        size_t len = src.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        buffer[len] = '\0';

        if (current_line != id) {
            tmp.println(buffer);
        }

        current_line++;
    }

    src.close();
    tmp.close();

    SD.remove(filename);
    SD.rename("/tmp.csv", filename);
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
                String name = String(file.name());
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