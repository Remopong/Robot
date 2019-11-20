#include <SPI.h> //Serial Peripheral Interface library
#include <NRFLite.h> //Library for NRF24L01 wireless modules
#include <I2Cdev.h> //Communication Library required
#include <MPU6050_6Axis_MotionApps20.h> //MPU6050 Library gyroscope
#include <Wire.h> //Communication Library required


//********************
#define YAW      0  //
#define PITCH    1  //   Define 3 Axis for the mpu6050 parameters
#define ROLL     2  //
//********************


//********************
int xAxe;           //
int yAxe;           //    Define 3 axis for personnal use
int zAxe;           //
//********************


//********************************************
bool dmpReady = false;                      //
uint8_t mpuIntStatus;                       //
uint8_t devStatus;                          //
uint16_t packetSize;                        //
uint16_t fifoCount;                         //
uint8_t fifoBuffer[64];                     //
Quaternion q;                               //   Variable used by the mpu6050
VectorFloat gravity;                        //
float ypr[3];                               //
volatile bool mpuInterrupt = false;         //
void dmpDataReady() {                       //
  mpuInterrupt = true;                      //
}                                           //
//********************************************


struct sender { // structure for NFR24L01: vars to send
  uint8_t xAxis;
  uint8_t yAxis;
  uint8_t zAxis;
};


//*****************
MPU6050 mpu;     //
NRFLite _radio;  //   Links between library/structs and vars
sender _Sender;  //
//*****************


//***********************************************
const static uint8_t RADIO_ID = 1;             //
const static uint8_t DESTINATION_RADIO_ID = 2; //   NRF24L01 Settings
const static uint8_t PIN_RADIO_CE = 9;         //
const static uint8_t PIN_RADIO_CSN = 10;       //
//***********************************************

void setup() {

  Serial.begin(9600);// Serial to 9600 bauds.

  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN)) { // If can't recognize the NRF24L01, then stay here.
    Serial.println("Can't connect to the radio");
    while (1);
  }

  TWBR = 24;
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize(); //MPU begins
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //Don't forget to calculate your personnal offsets, every gyroscope as differents one.

  //****************************
  mpu.setXAccelOffset(-3457); //
  mpu.setYAccelOffset(-808);  //
  mpu.setZAccelOffset(893);   //
  mpu.setXGyroOffset(260);    //    Setting up all the offsets
  mpu.setYGyroOffset(-49);    //
  mpu.setZGyroOffset(-32);    //
  //****************************


  if (devStatus == 0) { //Making tests
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



  //*************************************************************
  if (!dmpReady) {                                           //
    return;                                                  //
  }                                                          //
  while (!mpuInterrupt && fifoCount < packetSize) {}         //
  mpuInterrupt = false;                                      //
  mpuIntStatus = mpu.getIntStatus();                         //
  fifoCount = mpu.getFIFOCount();                            //
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {          //
    mpu.resetFIFO();                                         //
    Serial.println(F("FIFO overflow!"));                     //
  } else if (mpuIntStatus & 0x02) {                          //
    while (fifoCount < packetSize) {                         //
      fifoCount = mpu.getFIFOCount();                        //         Get the 3 Axis from the mpu6050
    }                                                        //
    mpu.getFIFOBytes(fifoBuffer, packetSize);                //
    fifoCount -= packetSize;                                 //
    mpu.dmpGetQuaternion(&q, fifoBuffer);                    //
    mpu.dmpGetGravity(&gravity, &q);                         //
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);               //
    xAxe = ypr[YAW] * (180 / M_PI);                          //
    yAxe = ypr[PITCH] * (180 / M_PI);                        //
    zAxe = ypr[ROLL] * (180 / M_PI);                         //
  //*************************************************************



    map(xAxe, -180, 180, 0, 180);
    map(yAxe, -180, 180, 0, 180); //Return our 3 axis from 0 to 180 instead of -180 to 180 for the servos.
    map(zAxe, -180, 180, 0, 180);

    Serial.print(xAxe);
    Serial.print("\t");
    Serial.print(yAxe);
    Serial.print("\t");
    Serial.println(zAxe);

    _Sender.xAxis = xAxe;
    _Sender.yAxis = yAxe;   //Fill up the Struct with the datas.
    _Sender.zAxis = zAxe;

    if (_radio.send(DESTINATION_RADIO_ID, &_Sender, sizeof(_Sender))) { //Sending the 3 Axis
      Serial.println("Message sent...");
    }
  }
}
