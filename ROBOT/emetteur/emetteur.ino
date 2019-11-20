#include <SPI.h>
#include <NRFLite.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>


#define YAW      0
#define PITCH    1
#define ROLL     2

MPU6050 mpu;

int xAxe;
int yAxe;
int zAxe;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;

void dmpDataReady() {
  mpuInterrupt = true;
}


struct sender {
  uint8_t xAxis;
  uint8_t yAxis;
  uint8_t zAxis;
};

NRFLite _radio;
sender _Sender;

const static uint8_t RADIO_ID = 1;
const static uint8_t DESTINATION_RADIO_ID = 2;
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

void setup() {

Serial.begin(9600);

  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN)) {
    Serial.println("Can't connect to the radio");
    while (1);
  }

  TWBR = 24;

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-3457);
  mpu.setYAccelOffset(-808);
  mpu.setZAccelOffset(893);
  mpu.setXGyroOffset(260);
  mpu.setYGyroOffset(-49);
  mpu.setZGyroOffset(-32);

  if (devStatus == 0) {

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0 : #pin2)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {

    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

}

void loop() {
  if (!dmpReady) {
    return;
  }

  while (!mpuInterrupt && fifoCount < packetSize) {}

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {

    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

  } else if (mpuIntStatus & 0x02) {

    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    xAxe = ypr[YAW] * (180 / M_PI);
    yAxe = ypr[PITCH] * (180 / M_PI);
    zAxe = ypr[ROLL] * (180 / M_PI);

    map(xAxe, -180, 180, 0, 180);
    map(yAxe, -180, 180, 0, 180);
    map(zAxe, -180, 180, 0, 180);

    Serial.print(xAxe);
    Serial.print("\t");
    Serial.print(yAxe);
    Serial.print("\t");
    Serial.println(zAxe);

    _Sender.xAxis = xAxe;
    _Sender.yAxis = yAxe;
    _Sender.zAxis = zAxe;

    if (_radio.send(DESTINATION_RADIO_ID, &_Sender, sizeof(_Sender))) {
      Serial.println("Message sent...");
    }
  }
}
