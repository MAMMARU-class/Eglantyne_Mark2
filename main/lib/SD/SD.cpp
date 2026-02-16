#include "SD.h"

SD::SD(){}

void SD::init(){
    if (!SD.begin()){
        delay(100);
    }

    // check Card type
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return;
    }
}

void SD::write_motion(
    const char* filename,
    array<float, 18> data
){
    File file = SD.open(filename, FILE_APPEND);
    if (!file) return;

    for (size_t i = 0; i < 18; ++i) {
        file.print(joint_angles[i], 6);
        if (i < 17) file.print(",");
    }
    file.println();

    file.close();
}

array<float, 18> SD::read_motion(
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

        if (current_line == line_number) {

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
    return result;
}
