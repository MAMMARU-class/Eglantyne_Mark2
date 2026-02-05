#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define OLED_SCL 47
#define OLED_SDA 21

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 400000UL, 100000UL);

void testdrawchar(uint8_t offset = 0) {
    display.clearDisplay();
    display.display();
    display.setTextSize(1);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);             // Start at top-left corner
    display.cp437(true);                 // Use full 256 char 'Code Page 437' font
    for (uint8_t i = 0; i < 170; i++)
        display.write((uint8_t)((i + offset) % 26 + 65));// 65:A to 90:Z
    display.display();
}

void setup()
{
    Wire.begin(OLED_SDA, OLED_SCL);

    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, false, false);

    display.clearDisplay();
    
    testdrawchar();
}

void loop()
{
    delay(100);
}