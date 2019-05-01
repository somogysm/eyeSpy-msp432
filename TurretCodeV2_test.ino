#define TroubleShoot false
#include <Enrf24.h>
#include <nRF24L01.h>
#include <string.h>
#include <SPI.h>
#include <Stepper.h>
#include <PID_v1.h>
// Core library for code-sense - IDE-based
#include "Energia.h"

// Include application, user and local libraries
#include "rtosGlobals.h"
#include "Event.h"

const int stepsPerRevolution=200;
int HorzstepCount=0; int VertstepCount=0; int intstate=LOW;
int Speed = 10;  int incomingByte;
int Directionx, Directiony;
int numbuffer;
boolean MODE1 = HIGH, MODE2 = !MODE1;
int16_t nh; int16_t nv; int16_t AccGyCo[9];
double Setpointx, Inputx, Outputx, OldOutputx = 0;
double Setpointy, Inputy, Outputy, OldInputy = 0;
double SetpointCamx, InputCamx, OutputCamx, OldInputCamx = 0;
double SetpointCamy, InputCamy, OutputCamy, OldInputCamy = 0;
double Kpx=0.05, Kix=0.5, Kdx=0;
double Kpy=0.03, Kiy=0, Kdy=0;
double KpCamx=0.25, KiCamx=0, KdCamx=0;
double KpCamy=0.5, KiCamy=0, KdCamy=0;
volatile int flag1= HIGH, flag2= HIGH, flag3=LOW;
const uint8_t rxaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };
Stepper HorzStepper(stepsPerRevolution, 40, 39, 38, 37);
Stepper VertStepper(stepsPerRevolution, 36, 35, 34, 33);
PID PIDx(&Inputx, &Outputx, &Setpointx, Kpx, Kix, Kdx, DIRECT);
PID PIDy(&Inputy, &Outputy, &Setpointy, Kpy, Kiy, Kdy, DIRECT);
PID PIDcamx(&InputCamx, &OutputCamx, &SetpointCamx, KpCamx, KiCamx, KdCamx, DIRECT);
PID PIDcamy(&InputCamy, &OutputCamy, &SetpointCamy, KpCamy, KiCamy, KdCamy, DIRECT);
Enrf24 radio(11,12,13); //CE CSN IRQ
int X, Y, xPos, yPos;
int red, green, blue;
void setup()
{
  pinMode(31, INPUT_PULLUP);
  pinMode(32, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  analogReadResolution(12);
  PIDx.SetMode(AUTOMATIC);
  PIDx.SetOutputLimits(-320,320);
  PIDy.SetMode(AUTOMATIC);
  PIDy.SetOutputLimits(-240,240);
  PIDcamx.SetMode(AUTOMATIC);
  PIDcamx.SetOutputLimits(-320,320);
  PIDcamy.SetMode(AUTOMATIC);
  PIDcamy.SetOutputLimits(-320,320);
  Setpointx=320;
  Setpointy=240;
  SetpointCamx=32;
  SetpointCamy=32;
  Serial.begin(9600);
  Serial1.begin(115200);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  radio.begin();
  radio.setRXaddress((void*)rxaddr);
  radio.enableRX();  // Start listening
  myEvent1.begin();
  myEvent2.begin();
  pinMode(PUSH2, INPUT_PULLUP);
  attachInterrupt(PUSH2, changeMode, CHANGE); // Interrupt is fired whenever button is pressed
  digitalWrite(78, MODE1);
}

void loop()
{
  if(flag3){
    MODE2=MODE1;
    MODE1=!MODE2;
    digitalWrite(78,MODE1);
    flag3=LOW;
    nv=0;
    nh=0;
  }
  if(MODE1){
  uint8_t inbuf[32]; //input buffer
 /* while (!radio.available(true))
  {
    Serial.println("No signal received");
    delay(500);
  }
  */
  if (radio.read(inbuf)){
    digitalWrite(BLUE_LED, HIGH);
    X=convertBytes(inbuf[0], inbuf[1]);
    Y=convertBytes(inbuf[2],inbuf[3]);
   /* for (int n=0; n<17; n+=2)
    {
      AccGyCo[n/2]=convertBytes(inbuf[n],inbuf[n+1]);
    }
    */
    digitalWrite(BLUE_LED, LOW);
  
    if (X!=-1){
    Inputx=X;
    PIDx.Compute();
    if (Inputx > Setpointx){
      Directionx = -1;
    }
    if (Inputx < Setpointx){
      Directionx = 1;
    }
    
    nh = (int)Directionx*abs(Outputx);
    if (abs(nh) < 1){
      nh = 1;
    }
    if (abs(X) < 1000){
    myEvent1.send();
    }
    }
    OldOutputx=Outputx;
    
    if (Y!=-1){
    Inputy=Y;
    PIDy.Compute();
    if (Inputy > Setpointy){
      Directiony = 1;
    }
    if (Inputy < Setpointy){
      Directiony = -1;
    }
    nv = (int)Directiony*abs(Outputy);
    //myEvent2.send();
    }
    OldInputy=Inputy;
    Serial.print(X);
    Serial.print("    ");
    Serial.println(nh);
    if (TroubleShoot){
      Serial.flush();
      Serial.print("Laser X: ");
      Serial.println(X, DEC);
      Serial.print("Laser Y: ");
      Serial.println(Y, DEC);
      Serial.print("PID Output X: ");
      Serial.println(Outputx);
      Serial.print("PID Output Y:  ");
      Serial.println(Outputy);
      Serial.print("Steps x: ");
      Serial.println(nh);
      Serial.print("Steps y: ");
      Serial.println(nv);
      delay(25);
     }
     radio.flush(); 
  }
  }//MODE_A CLOSING BRACKET

if (MODE2) {
    numbuffer = 0;
    int bytesToRead = 2;
    while(Serial.available() > 0){
      red=Serial.parseInt();
      green=Serial.parseInt();
      blue=Serial.parseInt();
          Serial1.write(red);
    Serial.println(red,HEX);
    Serial1.write(green);
    Serial.println(green,HEX);
    Serial1.write(blue);
    Serial.println(blue,HEX);
    delay(500);
    }

    while(Serial1.available() > 0 && bytesToRead > 0){
      int serialByte = Serial1.read();
      numbuffer += serialByte << ((bytesToRead-1)*8);
      bytesToRead = bytesToRead-1;
    }
    convertBits(numbuffer);
    
    InputCamx=xPos;
    InputCamy=yPos;
      PIDcamx.Compute();
      
      if (InputCamx < SetpointCamx){
        nh = abs(OutputCamx);
      }
      else if (InputCamx > SetpointCamx){
        nh = -abs(OutputCamx);
      }
      
      PIDcamy.Compute();
      if (InputCamy < SetpointCamy){
        nv = -abs(OutputCamy);
      }
      else if (InputCamy > SetpointCamy){
        nv = abs(OutputCamy);
      }
      myEvent1.send();
      myEvent2.send();
      Serial.println(nh);
    if (TroubleShoot){
      Serial.flush();
      Serial.print("Object X: ");
      Serial.println(xPos, DEC);
      Serial.print("Object Y: ");
      Serial.println(yPos, DEC);
      Serial.print("PID Output X: ");
      Serial.println(OutputCamx);
      Serial.print("PID Output Y: ");
      Serial.println(OutputCamy);
      Serial.print("X Steps: ");
      Serial.println(nh);
      Serial.print("Y Steps: ");
      Serial.println(nv);
      delay(50);
    }
  }                              //MODE_B CLOSING BRACKET
}

int16_t convertBytes(uint8_t msb, uint8_t lsb) //convert 2 bytes to 16 bit
{
  return (msb<<8) | lsb;
}

void convertBits(uint16_t value)
{
  xPos = value & 0xff;
  yPos = (value>>8) & 0xff;
}

void changeMode()
{

    flag3=HIGH;
}

