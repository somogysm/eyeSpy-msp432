///
/// @file		EMT_event.ino
/// @brief		Secondary sketch
///
/// Project EventLibrary for Energia MT 0101E0016
///

// Core library for code-sense - IDE-based
#include "Energia.h"

// Include application, user and local libraries

#include "rtosGlobals.h"
#include "Event.h"

void HorzMotorSetup()
{
  Serial.begin(115200);
  delay(500);
}


void HorzMotorLoop()
{
  myEvent1.waitFor();
  HorzStepper.setSpeed(Speed);
  HorzStepper.step(nh);
  HorzstepCount=HorzstepCount+nh;
  //nh=0;
  delay(50);
}

