#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <assert.h>

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

#define P3_MASK 0b0000000000011000
#define P1_MASK 0b0000000011111100
void setWriteDirInline() {
    //R_PORT3->PDR |= P3_MASK;
    //R_PORT1->PDR |= P1_MASK;
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);


    Serial.print("PORT3 dir: 0b");
    Serial.println(R_PORT3->PDR & P3_MASK, BIN);
    Serial.print("PORT1 dir: 0b");
    Serial.println(R_PORT1->PDR & P1_MASK, BIN);
}

void setReadDirInline() {
    //R_PORT3->PDR &= ~P3_MASK;
    //R_PORT1->PDR &= ~P1_MASK;
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    pinMode(5, INPUT);
    pinMode(6, INPUT);
    pinMode(7, INPUT);
    pinMode(8, INPUT);
    pinMode(9, INPUT);

    Serial.print("PORT3 dir: 0b");
    Serial.println(R_PORT3->PDR & P3_MASK, BIN);
    Serial.print("PORT1 dir: 0b");
    Serial.println(R_PORT1->PDR & P1_MASK, BIN);
}

uint8_t getPinMode(uint8_t pin) {
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);

    assert(port == 1 || port == 3);

    volatile uint16_t *reg;
    reg = portModeRegister(port);

    if (*reg & bit)
      return OUTPUT;
    else
      return INPUT;
}

void write8(uint8_t d) {
    assert(getPinMode(8) == OUTPUT);
    assert(getPinMode(9) == OUTPUT);
    assert(getPinMode(2) == OUTPUT);
    assert(getPinMode(3) == OUTPUT);
    assert(getPinMode(4) == OUTPUT);
    assert(getPinMode(5) == OUTPUT);
    assert(getPinMode(6) == OUTPUT);
    assert(getPinMode(7) == OUTPUT);

    /*R_PORT3->PODR = (R_PORT3->PODR & ~P3_MASK) | ((d&B00000001) << 4) |
                    ((d&B00000010) << 2);
    R_PORT1->PODR = (R_PORT1->PODR & ~P1_MASK) | ((d&B00000100) << 3) |
                    ((d&B00001000) << 1) | ((d&B00010000) >> 1) |
                    ((d&B00100000) >> 3) | (d&B11000000);*/

    digitalWrite(8, bitRead(d, 0));
    digitalWrite(9, bitRead(d, 1));
    digitalWrite(2, bitRead(d, 2));
    digitalWrite(3, bitRead(d, 3));
    digitalWrite(4, bitRead(d, 4));
    digitalWrite(5, bitRead(d, 5));
    digitalWrite(6, bitRead(d, 6));
    digitalWrite(7, bitRead(d, 7));

    Serial.print("PORT3 out: 0b");
    Serial.println(R_PORT3->PODR & P3_MASK, BIN);
    Serial.print("PORT1 out: 0b");
    Serial.println(R_PORT1->PODR & P1_MASK, BIN);

    digitalWrite(LCD_WR, LOW);
    delayMicroseconds(50);
    digitalWrite(LCD_WR, HIGH);
    delayMicroseconds(50);
}

uint8_t read8() {

    assert(getPinMode(8) == INPUT);
    assert(getPinMode(9) == INPUT);
    assert(getPinMode(2) == INPUT);
    assert(getPinMode(3) == INPUT);
    assert(getPinMode(4) == INPUT);
    assert(getPinMode(5) == INPUT);
    assert(getPinMode(6) == INPUT);
    assert(getPinMode(7) == INPUT);

    digitalWrite(LCD_RD, LOW);
    delayMicroseconds(50);

    Serial.print("PORT3 in: 0b");
    Serial.println(R_PORT3->PIDR & P3_MASK, BIN);
    Serial.print("PORT1 in: 0b");
    Serial.println(R_PORT1->PIDR & P1_MASK, BIN);

    /*uint8_t result = ((R_PORT3->PIDR & B00010000) >> 4) |
             ((R_PORT3->PIDR & B00001000) >> 2) |
             ((R_PORT1->PIDR & B00100000) >> 3) |
             ((R_PORT1->PIDR & B00010000) >> 1) |
             ((R_PORT1->PIDR & B00001000) << 1) |
             ((R_PORT1->PIDR & B00000100) << 3) |
             (R_PORT1->PIDR & B11000000) ;*/

    uint8_t result = digitalRead(8) | digitalRead(9) << 1 | digitalRead(2) << 2 | digitalRead(3) << 3 | digitalRead(4) << 4 | digitalRead(5) << 5 | digitalRead(6) << 6 | digitalRead(7) << 7;

    Serial.print("read byte: ");
    Serial.println(result, BIN);

    digitalWrite(LCD_RD, HIGH);
    delayMicroseconds(50);
    return result;
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("TFT LCD test"));

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  Serial.println(F("Using Adafruit 2.8\" TFT Arduino Shield Pinout"));
#else
  Serial.println(F("Using Adafruit 2.8\" TFT Breakout Board Pinout"));
#endif

  //Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());

  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);
  pinMode(LCD_CD, OUTPUT);
  digitalWrite(LCD_CD, HIGH);
  pinMode(LCD_WR, OUTPUT);
  digitalWrite(LCD_WR, HIGH);
  pinMode(LCD_RD, OUTPUT);
  digitalWrite(LCD_RD, HIGH);

  setWriteDirInline();


  /*tft.reset();

  uint16_t identifier = tft.readID();

  if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    return;
  }

  tft.begin(identifier);

  Serial.println(F("Benchmark                Time (microseconds)"));

  Serial.print(F("Screen fill              "));
  Serial.println(testFillScreen());
  delay(500);

  Serial.print(F("Text                     "));
  Serial.println(testText());
  delay(3000);

  Serial.print(F("Lines                    "));
  Serial.println(testLines(CYAN));
  delay(500);

  Serial.print(F("Horiz/Vert Lines         "));
  Serial.println(testFastLines(RED, BLUE));
  delay(500);

  Serial.print(F("Rectangles (outline)     "));
  Serial.println(testRects(GREEN));
  delay(500);

  Serial.print(F("Rectangles (filled)      "));
  Serial.println(testFilledRects(YELLOW, MAGENTA));
  delay(500);

  Serial.print(F("Circles (filled)         "));
  Serial.println(testFilledCircles(10, MAGENTA));

  Serial.print(F("Circles (outline)        "));
  Serial.println(testCircles(10, WHITE));
  delay(500);

  Serial.print(F("Triangles (outline)      "));
  Serial.println(testTriangles());
  delay(500);

  Serial.print(F("Triangles (filled)       "));
  Serial.println(testFilledTriangles());
  delay(500);

  Serial.print(F("Rounded rects (outline)  "));
  Serial.println(testRoundRects());
  delay(500);

  Serial.print(F("Rounded rects (filled)   "));
  Serial.println(testFilledRoundRects());
  delay(500);

  Serial.println(F("Done!"));*/
}

void checkId() {
  digitalWrite(LCD_CS, LOW);
  digitalWrite(LCD_CD, LOW);
  write8(0x04);
  setReadDirInline();
  digitalWrite(LCD_CD, HIGH);
  delayMicroseconds(50);

  uint8_t id1 = read8();
  uint8_t id2 = read8();
  uint8_t id3 = read8();
  uint8_t id4 = read8();

  digitalWrite(LCD_CS, LOW);
  setWriteDirInline();

  Serial.print("id1: 0b");
  Serial.println(id1, BIN);
  Serial.print("id2: 0b");
  Serial.println(id2, BIN);
  Serial.print("id3: 0b");
  Serial.println(id3, BIN);
  Serial.print("id4: 0b");
  Serial.println(id4, BIN);
}

void verifyPort(uint8_t pin, volatile uint16_t *outputRegister, uint16_t bitmask) {
    if(portOutputRegister(digitalPinToPort(pin)) == outputRegister) {
        Serial.println(String("pin ") + pin + " matches expected output register");
    } else {
        Serial.println(String("ERROR pin ") + pin + " does not match expected output register!");
    }

    if(digitalPinToBitMask(pin) != bitmask) {
        Serial.println(String("ERROR pin ") + pin + " does not match bitmask! e:" + bitmask + " a: " + digitalPinToBitMask(pin));
    }
}

#define write8inline(d)                                                        \
  {                                                                            \
    R_PORT1->PODR = (R_PORT1->PODR & ~P1_MASK) |                               \
                    ((d&0b0000000011000000) << 3) |                            \
                    ((d&0b0000000000111111) << 2);                             \
  }

void loop() {

  uint8_t tmp = 0b11111111;
  Serial.print("try: ");
  Serial.println(((tmp&0b0000000000111111) << 3), BIN);

  uint16_t tmp2 =
                    ((tmp&0b0000000011000000) << 3) |
                    ((tmp&0b0000000000111111) << 2);
  Serial.print("tmp2: ");
  Serial.println(tmp2, BIN);


  uint16_t identifier = tft.readID();
  Serial.print("id via lib: ");
  Serial.println(identifier, HEX);

  checkId();
  delay(1000);
  return;

  verifyPort(8, &(R_PORT3->PODR), 0b00010000);
  verifyPort(9, &(R_PORT3->PODR), 0b00001000);
  verifyPort(2, &(R_PORT1->PODR), 0b00100000);
  verifyPort(3, &(R_PORT1->PODR), 0b00010000);
  verifyPort(4, &(R_PORT1->PODR), 0b00001000);
  verifyPort(5, &(R_PORT1->PODR), 0b00000100);
  verifyPort(6, &(R_PORT1->PODR), 0b01000000);
  verifyPort(7, &(R_PORT1->PODR), 0b10000000);

  delay(100);
  return;

  /*uint16_t id = tft.readID();
  Serial.println(id, HEX);

  delay(100);
  return;
  tft.reset();

  uint16_t identifier = tft.readID();

  if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    return;
  }*/
}
