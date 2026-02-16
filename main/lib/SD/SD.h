#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define SDA 5
#define SCL 4

using std::array;

class SD{
public:
    SD()

    void init();

    void write_motion(
        const char* filename,
        array<float, 18>);
        
    array<float, 18> read_motion(
        const char* filename,
        size_t id);

private:

};
