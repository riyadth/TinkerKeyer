
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

/**
 * Function to read the paddle and key inputs, debounce them,
 * and generate events when the inputs change.
 */
void _read_paddle_and_key(void)
{
  uint8_t port_d = PIND & 0b00101100;
  uint8_t paddle = (port_d >> 2) & 0b00000011;
  uint8_t key = (port_d >> 6) & 0b00000001;

  // Debounce paddle and key inputs

}

/**
 * Function to read the encoder bits and return them to the caller.
 */
uint8_t _read_encoder(void)
{
  // Read the bits of port B, and mask out all but the encoder bits
  uint8_t encoder_bits = (PINB & 0b00001100) >> 2;

  // TODO: Read the encoder button also
  return encoder_bits;
}

/**
 * Read the encoder inputs and update the encoder "counter".
 * This routine must be called frequently enough to catch single-
 * bit changes in the encoder output signal.
 */
static void _handle_encoder(void)
{
  static uint8_t last_encoder_bits = 0;

  // Read the encoder bits and decide what to do with them
  uint8_t encoder_bits = _read_encoder();
  if (last_encoder_bits != encoder_bits)
  {
    // We have a "clock" and a "direction" bit to examine. Only evaluate
    // the new state if just one of them has changed.
  }
}

/**
 * Read the potentiometer attached to the board, and return the
 * raw 10-bit value to the caller.
 */
unsigned _read_potentiometer(void)
{
  return analogRead(A0);
}

/**
 * Process the buttons connected via resistor ladder to the
 * analog input.
 */
typedef enum {
  BUTTON_NONE = 0,
  BUTTON_1,
  BUTTON_2,
  BUTTON_3,
  BUTTON_4,
  BUTTON_5,
  BUTTON_MAX
} button_t;

// Macro to compute approximate voltage levels for comparison
#define V(x)  (unsigned)(((1023.0 * (float)(x)) / 5.0))

/**
 * Read the analog button input, and determine which (if any)
 * button is pressed, returning that value to the caller.
 */
unsigned _read_buttons(void)
{
  unsigned button_value = BUTTON_NONE;

  unsigned button_voltage = analogRead(A1);

  // Maximum voltage is 1023 (10 bits)

  if (button_voltage > V(4.5))
  {
    button_value = BUTTON_5;
  }
  else if (button_voltage > V(3.5))
  {
    button_value = BUTTON_4;
  }
  else if (button_voltage > V(2.5))
  {
    button_value = BUTTON_3;
  }
  else if (button_voltage > V(1.5))
  {
    button_value = BUTTON_2;
  }
  else if (button_voltage > V(0.5))
  {
    button_value = BUTTON_1;
  }

  return button_value;
}


/**
 * One-time setup function
 */
void setup() {
  Serial.begin(9600);
  while (!Serial)
  {
    // Busy wait!
  }

  lcd.begin();

  lcd.clear();

  lcd.print("Hello World!");

  // Configure digital inputs and outputs
  // The ICSP header (for the rotary encoder on port B
  pinMode(16, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  // The straight key input
  pinMode(6, INPUT_PULLUP);
  // The paddle inputs
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  // The keyed output signal
  digitalWrite(A2, LOW);
  pinMode(A2, OUTPUT);  // TODO: Need circuit to properly key the rig (optoisolator?)
  // The front panel LED
  digitalWrite(11, LOW);
  pinMode(11, OUTPUT);
  // The audio output
  digitalWrite(5, LOW);
  pinMode(5, OUTPUT);
}

/**
 * Repeated loop function
 */
void loop() {
  // Blink the on-board LED
  lcd.boardLED(true);
  delay(1000);
  lcd.boardLED(false);
  delay(1000);
}
