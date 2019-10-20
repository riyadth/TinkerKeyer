
/**
   TinkerKit LCD as a morse keyer

   Copyright Riyadth Al-Kazily, 2019
*/

/*
   Hardware information

   RXKED = PB0
   TXLED = PD5

   UART connector
     RX = 0
     TX = 1
   I2C/TWI connectors
     SDA = 2
     SCL = 3
   ICSP connector
     MOSI = 16
     MISO = 14
     SCK = 15

   Digital connectors
     5, 6, 11
   Analog connectors
     A0, A1, A2
*/

// Pin definitions
// Rotary encoder on ICSP header
#define ENC_A         16
#define ENC_B         14
#define ENC_BUTTON    15
// Paddle input on UART pins
#define PADDLE_DIT    0
#define PADDLE_DAH    1
// Key input
#define KEY_INPUT     A2
// Button resistor ladder
#define ANALOG_BUTTONS A1
// Potentiometer
#define POTENTIOMETER A0
// Front-panel LED
#define USER_LED      11
// Sidetone output
#define SIDETONE_PIN  5
// Keying output (to rig)
#define KEY_OUTPUT    6

/**
 * Minimal class to interface drive LCD module on TinkerKit LCD T01006x-05
 */
#include <LiquidCrystal.h>

class TinkerKit_LCD : public LiquidCrystal {
  public:
    TinkerKit_LCD() : LiquidCrystal(A4 /* RS */, A5 /* RW */, 7 /* EN */,
                                    A3 /* D4 */, 4 /* D5 */, 12 /* D6 */, 8 /* D7 */){};
    
    void begin() {
      boardLED(false);
      pinMode(13 /* BOARD_LED_PIN */, OUTPUT);
      LiquidCrystal::begin(16 /* Columns */, 2 /* Rows */);
      setBrightness(default_brightness);
      setContrast(default_contrast);
    }
    
    void setBrightness(uint8_t brightness) {
      analogWrite(10 /* BACKLIGHT_PIN */, brightness);
    }
    
    void setContrast(uint8_t contrast) {
      analogWrite(9 /* CONTRAST_PIN */, 255-contrast);
    }

    void boardLED(bool on) {
      digitalWrite(13 /* BOARD_LED_PIN */, on ? HIGH : LOW);
    }
    
    // Default settings for initialization
    const uint8_t default_brightness = 255;
    const uint8_t default_contrast = 230;
};

TinkerKit_LCD lcd = TinkerKit_LCD();

void setup() {
  Serial.begin(9600);
  while (!Serial)
  {
    // Busy wait!
  }

  lcd.begin();

  lcd.clear();

  lcd.print("Hello World!");

}

void loop() {
  // Blink the on-board LED
  lcd.boardLED(true);
  delay(1000);
  lcd.boardLED(false);
  delay(1000);
}
