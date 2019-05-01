#include <Wire.h>
#include <Enrf24.h>
#include <nRF24L01.h>
#include <string.h>
#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 accelgyro;

const int MPU_addr=0x68; 
int16_t aX, mx, my, mz, Tmp;
uint8_t txBuf[32]={}; //transmission buffer
Enrf24 radio(11, 12, 13); //CE CSN IRQ
int16_t vSteps=0; int16_t hSteps=0;
uint8_t x1Low, x1High, y1Low, y1High, Red, Green, Blue; //for seperating 16 bit data into 2 bytes
uint8_t x2Low, x2High, y2Low, y2High;
uint8_t xLow, xHigh, yLow, yHigh;
uint8_t xsetpointLow = 0, xsetpointHigh = 0;
uint8_t ysetpointLow = 0, ysetpointHigh = 0;
const uint8_t txaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 }; //RF address
int button[5] = {40, 39, 38, 37, 36}; // Pins 2.7, 2.6, 2.4, 5.6, and 6.6 are for buttons.
int  eyeBuffer, startTime = 0, bounceTime = 5000;
boolean serialReceived = false;
boolean eyeMode = true;
volatile int flag1 = LOW, flag2 = LOW, flag3 = LOW, flag4 = LOW, flag5 = LOW;
int laserBuffer[7];

void setup()
{
  Wire.begin(); //I2C for sensors
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); //PWR MGMT 1 register
  Wire.write(0); //wake up MPU6050
  Wire.endTransmission(true);
  Serial.begin(115200);
  Serial1.begin(115200);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  radio.begin();
  radio.setTXaddress((void*)txaddr);
  accelgyro.initialize();
 for (int j=0; j < 6; j ++)
 {
   pinMode(button[j], INPUT_PULLUP);
 }
 attachInterrupt(button[0], checkButtons1, FALLING);
 attachInterrupt(button[1], checkButtons2, FALLING);
 attachInterrupt(button[2], checkButtons3, FALLING);
 attachInterrupt(button[3], checkButtons4, FALLING);
 attachInterrupt(button[4], checkButtons5, FALLING);
 pinMode(35,INPUT_PULLUP);
}



void loop()//DONT PRINT STUFF INSIDE SERIAL FUNCTIONS
{
  for (int j = 0; j < 32; j++) txBuf[j] = 0; // Clear TX buffer.
  
  //Laser
  if (Serial1.available() > 0){
    digitalWrite(RED_LED, HIGH);
    laserBuffer[7] = {0};
    int bytesToRead = 7; 
    int k = 0;
    while(Serial1.available() > 0 && bytesToRead > 0 ){
      int serialByte = Serial1.read();
      laserBuffer[k] = serialByte; //<< ((bytesToRead-1)*8);
      bytesToRead = bytesToRead-1;
      k++;
    }
  //convertBits1(laserBuffer>>24 & 0xFFFFFFFF);
  //convertBits3(laserBuffer & 0xFFFFFF);
  serialReceived = true;
  }
  
  txBuf[16] = digitalRead(35);
  //digitalWrite(GREEN_LED, q);

  
  if (eyeMode){
    if (Serial.available() > 0){
      //digitalWrite(GREEN_LED, HIGH);
      eyeBuffer = 0;
      int bytesToRead = 4;
      while(Serial.available() > 0 && bytesToRead > 0){
        int serialByte = Serial.read();
        eyeBuffer += serialByte << ((bytesToRead-1)*8);
        bytesToRead = bytesToRead-1;
      }
      serialReceived = true;
    }
    //xsetpointLow = 64, xsetpointHigh = 1;
    //ysetpointLow = 240, ysetpointHigh = 0;
    convertBits2(eyeBuffer);
    
  }
 
 if (serialReceived)
  {
    digitalWrite(BLUE_LED,HIGH);
    txBuf[0]=laserBuffer[0];
    txBuf[1]=laserBuffer[1];
    txBuf[2]=laserBuffer[2];
    txBuf[3]=laserBuffer[3];
    txBuf[4]=laserBuffer[4];
    txBuf[5]=laserBuffer[5];
    txBuf[6]=laserBuffer[6];
    txBuf[17]=x2High;
    txBuf[18]=x2Low;
    txBuf[19]=y2High;
    txBuf[20]=y2Low;
  }
    
    if (flag1 == HIGH && isDebounced(millis())){
      digitalWrite(GREEN_LED,HIGH);
      txBuf[7] = button[0];
      //Serial.println(button[0]);
      flag1=LOW;
      delay(500);
    } 
    else if (flag2 == HIGH && isDebounced(millis())){
      digitalWrite(GREEN_LED,HIGH);
      txBuf[7] = button[1];  // Write button pin if pressed.
      Serial.println(button[1]);
      //eyeMode = !eyeMode;// Change Eye Mode
      flag2=LOW;
      delay(500);
    }
    else if (flag3 == HIGH && isDebounced(millis())){
      digitalWrite(GREEN_LED,HIGH);
      txBuf[7] = button[2];  // Write button pin if pressed.
      //Serial.println(button[2]);
      flag3=LOW;
      delay(500);
    }
     else if (flag4 == HIGH && isDebounced(millis())){
      digitalWrite(GREEN_LED,HIGH);
      txBuf[7] = button[3];
      //Serial.println(button[3]);  // Write button pin if pressed.
      flag4=LOW;
      delay(500);
    }
    else if (flag5 == HIGH && isDebounced(millis())){
      digitalWrite(GREEN_LED,HIGH);
      txBuf[7] = button[4];
      flag5=LOW;
      delay(500);
    }
    
  writeAcceleration();
  writeCompass();
  sendRF();
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  //digitalWrite(GREEN_LED, LOW);
  Serial1.flush();
  Serial.flush();
}

void convertBits1(int32_t value) //convert 16 bit to two bytes for transmission
{
   y1Low = (value) & 0xff;
   y1High = (value>>8) & 0xff;
   x1Low = (value>>16) & 0xff;
   x1High = (value>>24) & 0xff;
}

void convertBits2(int32_t value)
{
   y2Low = value & 0xff; 
   y2High = (value>>8) & 0xff; 
   x2Low = (value>>16) & 0xff;
   x2High = (value>>24) & 0xff;
}

void convertBits3(int32_t value)
{
   Blue = value & 0xff;
   Green = (value>>8) & 0xff;
   Red = (value>>16) & 0xff;
}

/*void combineBytes()
{
  xLow = x1Low + (xsetpointLow - x2Low);
  yLow = y1Low + (ysetpointLow - y2Low);
  xHigh = x1Low + (xsetpointHigh - x2High);
  yHigh = y1High + (ysetpointHigh - y2High);
}*/

void writeAcceleration()
{
  aX = accelgyro.getAccelerationX();
  txBuf[8] =(aX >> 8) & 0xFF ;
  txBuf[9] =aX & 0xFF ; 
}

void writeCompass()
{
  accelgyro.getMag(&mx, &my, &mz);
  mx = mx - 49; // Calibration offset.
  txBuf[10] = (mx >> 8) & 0xFF ;
  txBuf[11] = mx & 0xFF ; 
  txBuf[12] = (my >> 8) & 0xFF ;
  txBuf[13] = my & 0xFF ; 
  txBuf[14] = (mz >> 8) & 0xFF ;
  txBuf[15] = mz & 0xFF ; 
}

void sendRF()
{
  for (int c = 0; c < 32; c++) radio.write(txBuf[c]);
  radio.flush();
}

boolean isDebounced(int now)
{
  if (now - startTime > bounceTime) // Only accept button presses if debounce time has passed.
  {
    startTime = millis();
    return true;
  }
  else return false;
}

void checkButtons1()
{
  flag1 = HIGH;
}

void checkButtons2()
{
  flag2 = HIGH;
}

void checkButtons3()
{
  flag3 = HIGH;
}

void checkButtons4()
{
  flag4 = HIGH;
}

void checkButtons5()
{
  flag5 = HIGH;
}
