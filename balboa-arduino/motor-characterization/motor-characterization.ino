// This program shows how to read the encoders on the Balboa 32U4.
// The encoders can tell you how far, and in which direction each
// motor has turned.
//
// You can press button A on the Balboa to drive both motors
// forward at full speed.  You can press button C to drive both
// motors in reverse at full speed.
//
// Encoder counts are printed to the LCD and to the serial
// monitor.
//
// On the LCD, the top line shows the counts from the left
// encoder, and the bottom line shows the counts from the right
// encoder.  Encoder errors should not happen, but if one does
// happen then the buzzer will beep and an exclamation mark will
// appear temporarily on the LCD.
//
// In the serial monitor, the first and second numbers represent
// counts from the left and right encoders, respectively.  The
// third and fourth numbers represent errors from the left and
// right encoders, respectively.

#include <Balboa32U4.h>

Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4Motors motors;
Balboa32U4ButtonA buttonA;
Balboa32U4ButtonC buttonC;

const char encoderErrorLeft[] PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";

char report[90];

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  static uint32_t lastDisplayTime;
  static uint8_t displayErrorLeftCountdown = 0;
  static uint8_t displayErrorRightCountdown = 0;


  if (millis() >= 10) {
    //motors.setSpeed(
  }

  if ((uint32_t)(millis() - lastDisplayTime) >= 10)
  {
    lastDisplayTime = millis();

    int16_t countsLeft = encoders.getCountsLeft();
    int16_t countsRight = encoders.getCountsRight();

    bool errorLeft = encoders.checkErrorLeft();
    bool errorRight = encoders.checkErrorRight();

    // Send the information to the serial monitor also.
    snprintf_P(report, sizeof(report),
        PSTR("%13d \t %6d \t %6d \t %6d"), (uint16_t)lastDisplayTime, readBatteryMillivolts(),
        countsLeft, countsRight);
    Serial.println(report);
  }

  if (buttonA.isPressed())
  {
    motors.setSpeeds(300, 300);
  }
  else if (buttonC.isPressed())
  {
    motors.setSpeeds(-300, -300);
  }
  else
  {
    motors.setSpeeds(0, 0);
  }
}
