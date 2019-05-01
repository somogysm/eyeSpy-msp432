#define TroubleShoot true
#define Mode1 false
#define Mode2 true
#include <Enrf24.h>
#include <nRF24L01.h>
#include <string.h>
#include <SPI.h>
#include <Stepper.h>
#include <PID_v1.h>
const int stepsPerRevolution=200;
int HorzstepCount=0; int VertstepCount=0; int intstate=LOW;
int Speed = 120;  int incomingByte;
int Direction;
int numbuffer;
int16_t nh; int16_t nv; int16_t AccGyCo[9];
double Setpointx, Inputx, Outputx;
double SetpointCamx, InputCamx, OutputCamx;
double OldInputx = 0, OldOutputx = 0;
double OldInputCamx = 0, OldOutputCamx = 0;
double Kpx=1, Kix=1, Kdx=1;
double KpCamx=1, KiCamx=1, KdCamx=1;
volatile int flag1= HIGH; volatile int flag2= HIGH;
const uint8_t rxaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };
Stepper HorzStepper(stepsPerRevolution, 40, 39, 38, 37);
Stepper VertStepper(stepsPerRevolution, 36, 35, 34, 33);
PID PIDx(&Inputx, &Outputx, &Setpointx, Kpx, Kix, Kdx, DIRECT);
PID PIDcamx(&InputCamx, &OutputCamx, &SetpointCamx, KpCamx, KiCamx, KdCamx, DIRECT);
Enrf24 radio(11,12,13); //CE CSN IRQ
int X, Y, xPos, yPos;

void setup()
{
  pinMode(31, INPUT_PULLUP);
  pinMode(32, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  analogReadResolution(12);
  PIDx.SetMode(AUTOMATIC);
  PIDcamx.SetMode(AUTOMATIC);
  PIDcamx.SetOutputLimits(0,64);
  PIDx.SetOutputLimits(0,640);
  Setpointx=320;
  SetpointCamx=32;
  if (TroubleShoot){
    Serial.begin(9600);
  }
  Serial1.begin(115200);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  radio.begin();
  radio.setRXaddress((void*)rxaddr);
  radio.enableRX();  // Start listening
}

void loop()
{
  if (Mode1){
  uint8_t inbuf[32]; //input buffer
  while (!radio.available(true))
  {
    ;//do nothing
  }
  if (radio.read(inbuf)){
    digitalWrite(BLUE_LED, HIGH);
    X=convertBytes(inbuf[0], inbuf[1]);
    Y=convertBytes(inbuf[2],inbuf[3]);
    radio.flush();
    /*
    for (int n=0; n<17; n+=2)
    {
      AccGyCo[n/2]=convertBytes(inbuf[n],inbuf[n+1]);
    }
    */
    if (X!=-1){
      Inputx=X;
      PIDx.Compute();
      if (Outputx > OldOutputx){
        nh = 1;
      }
      else if (Outputx < OldOutputx){
        nh = -1;
      }
      else{
        nh = 0;
      }
      //nh = Direction;//*abs(Outputx-Setpointx)/10;
    }
    if (TroubleShoot){
      Serial.flush();
      Serial.print("Laser X: ");
      Serial.println(X, DEC);
      Serial.print("Laser Y: ");
      Serial.println(Y, DEC);
      Serial.print("PID Output X: ");
      Serial.println(Outputx);
      Serial.print("Steps: ");
      Serial.println(nh);
      delay(25);
    }
     digitalWrite(BLUE_LED, LOW);
    }
  //if (!(nv == 0)) //receive horizontal step input
  //Vertfullstep(); //rotate horizontal steppers
  if (abs(nh) && !(X==-1)){
    Horzfullstep();
  }
  OldInputx=Inputx;
  OldOutputx=Outputx;
}
  
  if (Mode2) {
    numbuffer = 0;
    int bytesToRead = 2;
    while(Serial1.available() > 0 && bytesToRead > 0){
      int serialByte = Serial1.read();
      numbuffer += serialByte << ((bytesToRead-1)*8);
      bytesToRead = bytesToRead-1;
    }
    convertBits(numbuffer);
    
    InputCamx=xPos;
      PIDcamx.Compute();
      if (InputCamx < SetpointCamx){
        nh = abs(OutputCamx-SetpointCamx);
      }
      else if (InputCamx > SetpointCamx){
        nh = -abs(OutputCamx-SetpointCamx);
      }
      else{
        nh = 0;
      }
      
    if (TroubleShoot){
      Serial.flush();
      Serial.print("Object X: ");
      Serial.println(xPos, DEC);
      //Serial.print("Object Y: ");
      //Serial.println(yPos, DEC);
      //Serial.print("PID Output X: ");
      //Serial.println(OutputCamx);
      Serial.print("Steps: ");
      Serial.println(nh);
      delay(50);
    }
    
    if (abs(nh)){
      Horzfullstep();
    }
    OldInputCamx=InputCamx;
    OldOutputCamx=OutputCamx;
  /*
  int numbuffer = 0;
  int bytesToRead = 2;
  while(Serial1.available() > 0 && bytesToRead > 0){
    int serialByte = Serial1.read();
    numbuffer += serialByte << ((bytesToRead-1)*8);
    bytesToRead = bytesToRead-1;
  }
 
  Serial.print((numbuffer>>8)&0xff,DEC);
  Serial.print("    ");
  Serial.println(numbuffer & 0xff,DEC);
  delay(50);*/
  }
}
void Vertfullstep(){
  VertStepper.setSpeed(Speed);
  VertStepper.step(nv);
  VertstepCount=VertstepCount+nv;
  nv=0;
}
void Horzfullstep(){
  HorzStepper.setSpeed(Speed);
  HorzStepper.step(nh);
  HorzstepCount=HorzstepCount+nh;
  nh=0;
}

int16_t convertBytes(uint8_t msb, uint8_t lsb) //convert 2 bytes to 16 bit
{
  return (msb<<8) | lsb;
}

void convertBits(int16_t value) //convert 16 bit to two bytes for transmission
{
   xPos = value & 0xff; //lowest 8 bits
   yPos = (value>>8) & 0xff; //highest 8 bits
}
