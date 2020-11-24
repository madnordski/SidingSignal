/** SidingSignal
    Uses the Arduino Nano Every, the OLED1331, and the ToF10120 to
    display distance the train is from the end of a siding.  It also
    uses a reed switch to determine if the siding is open or closed to
    trains.
    
    The 0.95 inch OLED SSD1331 RGB Screen is 96 x 64 pixels and uses
    the SPI interface.

    NB: to convert bitmap or png to code use:
        On my system use: Documents/Arduino/image2cpp-master/index.html
	All others use: http://javl.github.io/image2cpp/
	
    NB: the program PikoPixel was used to create the bitmaps.

    See images.h for images used by this program.

    Author: Joseph King
    Copyright (c) 2020, Joseph King.

    Artistic License 2.0: Everyone is permitted to copy and distribute
    verbatim copies of this license document, but changing it is not
    allowed.  See https://poc-library.readthedocs.io/en/release/References/Licenses/ArtisticLicense2.0.html

*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <math.h>

#include "images.h"

// pin defines for OLED SPI connection
#define SCLK_PIN 13
#define MOSI_PIN 11
#define RST_PIN 3
#define DC_PIN 5
#define CS_PIN 4

// RGB LED pin defines
#define REDPIN 10 // rgb led red
#define GRNPIN  9 // rgb led green
#define BLEPIN  6 // rgb led blue

// the reed switch is on this pin
#define SWITCHPIN 2

// use hardware SPI interface (uses all pins, just these are passed)
Adafruit_SSD1331 display(&SPI, CS_PIN, DC_PIN, RST_PIN);

// define colors for convenience
#define BLACK           0x0000
#define BLUE            0x001F
#define DARKBLUE        0x000B
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF
#define GREY            0x8410
#define ORANGE          0xE880
#define DARKORANGE      0x3880

// This code works with one for two TOFs we previously programmed to
// have different I2C addresses.  Note: you must use the serial
// interface to program the I2C address and this code is not included
// here.  In our case, we built two SidingSignals -- TOF1 is at
// the end of siding 5 and TOF2 is at the end of siding 6 on our
// layout.

#define TOF1 83  // number 1 is 83, number 2 is 82
#define TOF2 82

// reed switch open or closed -- nb: this can mean different things
// for each siding
#define CLOSED LOW
#define OPEN   HIGH

// the TOFs as deployed have different usable ranges starting at the begin
// value below and ending at either 6 (TOF1) or 2 (TOF2), offset is
// the difference between starting values (use offset only if
// consistency between siding distances is important).
#define BEGIN1 25
#define BEGIN2 40
#define OFFSET1 4 // not in use
#define OfFSET2 0

// globals

// the TOF used in this running instance of the program and associated values
int TOF = TOF1;
int beginZone = BEGIN1;
int offsetReading = OFFSET1;

// reuse the read buffer and keep track of previous readings to avoid
// unnessary updates
unsigned char ToFBuf[16];
int lastMM = 0;
unsigned long lastTime = 0;

// color by distance table -- scale feet to color (red, yellow, green)
uint16_t ColorTable[]
// 0       1       2       3       4       5       6      7      9       9
= {RED,    RED,    RED,    RED,    RED,    RED,    RED,   RED,   YELLOW, YELLOW,
   YELLOW, YELLOW, YELLOW, YELLOW, YELLOW, GREEN};
uint16_t LastColor = BLACK;

/** sidingIsClosed
    Return true if the siding is closed to trains.
*/
bool sidingIsClosed() {
  if ( TOF == TOF2 )
    return(digitalRead(SWITCHPIN) == CLOSED);
  else
    return(digitalRead(SWITCHPIN) == OPEN);
}

// start the display of numbers -- differnt for each tof1
int begin1, begin2;

/** setup
    Start things up.  Our RGB LED is purple until a TOF is selected
    (TOF1 or TOF2) depending on which is connected.
    The RGB LED turns green when TOF1 is selected and yellow for TOF2.
    After a short delay, the LED turns off and is subsequently
    controlled by the loop.
*/

void setup() {
  Wire.begin(); // I2C startup
  
  Serial.begin(9600);
  display.begin();

  // rgb led
  pinMode(REDPIN, OUTPUT);
  pinMode(GRNPIN, OUTPUT);
  pinMode(BLEPIN, OUTPUT);
  setColor(255, 0, 255);
  
  pinMode(SWITCHPIN, INPUT_PULLUP);
  
  display.setRotation(2);
  // test images
  if ( 1 ) {
    display.fillScreen(BLACK);
    display.drawBitmap(0, 0, NoEntry, 96, 64, RED);
    delay(1000);
    display.fillScreen(BLACK);
    display.drawBitmap(0, 0, Go, 96, 64, GREEN);
    delay(1000);
    display.fillScreen(BLACK);
    display.drawBitmap(0, 0, Warning, 96, 64, YELLOW);
    delay(1000);
    display.fillScreen(BLACK);
    display.drawBitmap(0, 0, Stop, 96, 64, RED);
    delay(1000);
  }
  display.fillScreen(BLACK);
  display.drawBitmap(0, 0, LionelGrey, 96, 64, GREY);
  display.drawBitmap(0, 0, Lionel, 96, 64, DARKBLUE);
  display.drawBitmap(0, 0, LionelOrange, 96, 64, DARKORANGE);

  display.setTextColor(WHITE, BLACK);

  // TRY FINDING THE TOF CONNECTED
  TOF = tofRead(TOF1) != -1 ? TOF1 : TOF2;
  if ( TOF == TOF1 ) {
    Serial.println("TOF1 Selected");
    setColor(0, 255, 0); // green for tof 1
    beginZone = BEGIN1;
  }
  else {
    Serial.println("TOF2 Selected");
    setColor(255, 255, 0); // yellow for tof 2
    beginZone = BEGIN2;
  }
  // let's just pause and then turn off the led because it distracts
  delay(2000);
  setColor(0, 0, 0);
  Serial.println("Ready.");
}

// these globals are used by the loop
int16_t TxtX = 28;  // pixel starting position for text
int16_t TxtY = 22;
bool SidingWasClosed = true; // state of the sliding (closed to trains)

/** loop
    Read the ToF, if the siding is open use the distance returned by
    the ToF to update the display.  The display is blank when the
    distance is greater than begin and green, yellow or red if less
    than or equal to begin.  Right now the red and yellow zones are
    hard coded.
*/

void loop() {
  int d1 = scaleFeetFrom(TOF);
  uint16_t fgColor;
  uint16_t thisColor;

  // did we get a reading
  if ( d1 == -1 ) {
    if ( TOF == TOF1 ) {
      setColor(255, 0, 255); // purple
      Serial.print("Unable to read TOF1 on siding 5");
    }
    else {
      setColor(255, 165, 0); // orange
      Serial.print("Unable to read TOF2 on siding 6");
    }
    delay(1000);
    return; // keep trying forever
  }
  
  // set the color zone

  if ( d1 < 16 )
    thisColor = ColorTable[d1];
  else if ( d1 <= beginZone )
    thisColor = GREEN;
  else
    thisColor = BLACK;  

  // check the siding state at the beginning of the loop and go with it
  
  bool sidingClosed = sidingIsClosed();
  //Serial.println(sidingClosed ? "Siding is closed." : "Siding is open.");

  // has the siding state changed
  if ( sidingClosed != SidingWasClosed ) {
    // yes, changed to closed
    if ( sidingClosed ) {
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, NoEntry, 96, 64, RED);
      setColor(0, 0, 0); // siding closed
    }
    // yes, changed to open
    else {
      LastColor = BLACK;
      display.fillScreen(BLACK);
      setColor(0, 255, 0); // siding open
    }
  }

  // use the color to determine if the screen needs an update
  if ( !sidingClosed && thisColor != LastColor ) {
    
    // siding is open, update using the new color and symbol
    display.fillScreen(BLACK);
    switch (thisColor) {
    case GREEN:
      display.drawBitmap(0, 0, Go, 96, 64, GREEN);
      TxtX = 28;
      TxtY = 22;
      setColor(0, 255, 0);
      break;
    case YELLOW:
      display.drawBitmap(0, 0, Warning, 96, 64, YELLOW);
      TxtX = 28;
      TxtY = 32;
      setColor(255, 255, 0);
      break;
    case RED:
      TxtX = 22;
      TxtY = 22;
      setColor(255, 0, 0);
      display.drawBitmap(0, 0, Stop, 96, 64, RED);
      break;
    }
    // remember the color/symbol chosen
    LastColor = thisColor;
  }

  // finally, if the siding is open print the scale feet
  if ( ! sidingClosed ) {
    fgColor = thisColor == BLACK ? WHITE : BLACK;
    display.setTextColor(fgColor, thisColor);
    display.setTextSize(3);
    display.setCursor(TxtX, TxtY);
    // only print scale feet if the number is lte to begin
    if ( d1 <= beginZone ) {
      if ( d1 < 10 ) display.print(" ");
      display.print(d1);
    }
    delay(30);
  }
  // remember the siding state
  SidingWasClosed = sidingClosed;
}

/** tofRead
    Read mm distance from the tof.
    @param i2cAddr the I2C address of the tof to read.
    @return mm distance or -1 if it times out.
*/

int tofRead(int i2cAddr) {
  byte buf[2];
  int result = 0;

  // read echos
  Wire.beginTransmission(i2cAddr);

  // data address
  Wire.write(0x00);
  Wire.endTransmission();

  // datasheet says wait 30uS
  delay(1);

  // request two bytes
  Wire.requestFrom(i2cAddr, 2);

  // wait for the response
  //  with error timeout
  unsigned long start = millis();
  while ( Wire.available() < 2 ) {
    if ( millis() - start > 300 ) {
      result = -1;
      break;
    }
  }

  // no error, read and pack result
  if ( result == 0 ) {
    // read two bytes
    for (int i=0; i<2; i++)
      buf[i] = Wire.read();

    // stuff the answer into an integer
    result = buf[0];
    result = result << 8;
    result |= buf[1];
  }
  
  return(result);
}

/** scaleFeetFrom
    Read scale feet from the tof.
    @param i2cAddr the address of the tof to read.
    @return scale feet or -1 if there's an error.
*/

int scaleFeetFrom(int i2cAddr) {
  int sum = 0;
  int feet;
  int reading;
  for (int i=0; i<50; i++) {
    reading = tofRead(i2cAddr);
    if ( reading == -1 ) return(-1);
    sum += reading;
  }
  feet = round((sum/50.0) * 0.1378);
  return(feet);
}

/** setColor
    Set the color of the RGB LED.
    @param red value for red (0 to 255)
    @param green value for green
    @param blue value of blue
*/

void setColor(int red, int green, int blue) {
  analogWrite(REDPIN, red);
  analogWrite(GRNPIN, green);
  analogWrite(BLEPIN, blue);
}  
