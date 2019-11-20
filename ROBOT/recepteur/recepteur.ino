#include <Servo.h>
#include <NRFLite.h>

const static uint8_t RADIO_ID = 2;
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

int angleServoX;
int angleServoY;
int angleServoZ;

struct receiver {
  uint8_t xAxis;
  uint8_t yAxis;
  uint8_t zAxis;
};

NRFLite _radio;
receiver _Receiver;
Servo ServoX;
Servo ServoY;
Servo ServoZ;

void setup() {
  Serial.begin(9600);
  ServoX.attach(3);
  ServoY.attach(5);
  ServoZ.attach(6);

  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN)) {
    Serial.println("radio error");
    while (1);
  }
}

void loop() {
  while (_radio.hasData())
  {
    _radio.readData(&_Receiver); // Note how '&' must be placed in front of the variable name.

    String msg = "Radio ";
    msg += _Receiver.xAxis;
    angleServoX = _Receiver.xAxis;
    ServoX.write(angleServoX);
    msg += ", ";
    msg += _Receiver.yAxis;
    angleServoY = _Receiver.yAxis;
    ServoY.write(angleServoY);
    msg += " , ";
    msg += _Receiver.zAxis;
    angleServoZ = _Receiver.zAxis;
    ServoZ.write(angleServoZ);

    Serial.println(msg);
  }
}
