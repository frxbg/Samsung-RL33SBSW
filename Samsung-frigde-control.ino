/*
  Управление за хладилник модел: rl33sbsw 
  от Тони Стоянов
*/
#include <math.h>
#include "MegunoLink.h"
#include "Filter.h"

// Create a new exponential filter with a weight of 10 and initial value of 0.
ExponentialFilter<long> ADCFilter(30, 0);

//Дефинираме входовете
int FreezeSensor;
int EvapSensor;
int FridgeSensor;
int DoorSwitch = A3;

//Дефинираме изходите
int DefrostHeater = 1;
int Compressor = 2;
int FanMotor = 3;
int InsideLamp = 4;
int GearMotor = 5;

//Дефинираме променливите
int FreezeSet = -18;
int FridgeSet = 6;
int doorCount;
bool Defrosting = 0;

unsigned long StartDelay;
unsigned long DefrostInterval;
unsigned long DefrostTime;
unsigned long DriptingTime;
unsigned long DoorAlarm;

void setup() {
  
  // declare the ledPin as an INPUT:
  pinMode(DoorSwitch, INPUT);
  // declare the ledPin as an OUTPUT:
  pinMode(DefrostHeater, OUTPUT);
  pinMode(Compressor, OUTPUT);
  pinMode(FanMotor, OUTPUT);
  pinMode(InsideLamp, OUTPUT);
  pinMode(GearMotor, OUTPUT);

  digitalWrite(DefrostHeater, LOW);
  digitalWrite(Compressor, LOW);
  digitalWrite(FanMotor, LOW);
  digitalWrite(InsideLamp, HIGH);
  digitalWrite(GearMotor, LOW);

  StartDelay = millis() + 120000;
  DefrostInterval = millis + 14400000;
}

// *********************************************************************
// loop
// *********************************************************************
void loop() {

  if (FreezeSensor <= 4 && FridgeSensor <= 6 && doorCount <= 1) {
    DefrostInterval = millis + 68400000;
  }
  else {
    DefrostInterval = millis + 2160000;
  }
  if (FreezeSensor >= (FreezeSet + 2) && millis() > StartDelay && Defrosting == 0) {
    CompressorStart();
  }
  if (FreezeSensor <= FreezeSet) {
    CompressorStop();
  }

  if (millis() > DefrostInterval) {
    DefrostControl;
  }

  DoorControl();
  GetFreezeTemp();
  GetEvapSensor();
  GetFridgeSensor();
  FridgeTempControl();
}

// *********************************************************************
// FridgeTempControl
// *********************************************************************
void FridgeTempControl()
{
  if (Defrosting  == 0 && FridgeSensor >= FridgeSet + 2) {
    digitalWrite(GearMotor, HIGH);
  }
  if (FridgeSensor < FridgeSet - 2 || digitalRead(DoorSwitch == HIGH)) {
    digitalWrite(GearMotor, LOW);
  }
}

// *********************************************************************
// UpdateLeds
// *********************************************************************
void UpdateLeds()
{

}
// *********************************************************************
// DoorControl
// *********************************************************************
void DoorControl()
{
  if (digitalRead(DoorSwitch == HIGH)) {
    digitalWrite(InsideLamp, HIGH);
    doorCount = doorCount + 1;
    DoorAlarm = millis() + 120000;
  }
  else if (digitalRead(DoorSwitch == LOW)) {
    digitalWrite(InsideLamp, LOW);
    DoorAlarm = 0;
  }
}
// *********************************************************************
// DefrostControl
// *********************************************************************
void DefrostControl()
{
  if (EvapSensor <= -5 && Defrosting == 0) {
    Defrosting = 1;
    DefrostTime  = millis() + 4200000;
    digitalWrite(DefrostHeater, HIGH);
    digitalWrite(Compressor, LOW);
    digitalWrite(GearMotor, LOW);
  }
  if (EvapSensor >= 12 || millis() > DefrostTime) {
    digitalWrite(DefrostHeater, LOW);
    DriptingTime = millis() + 420000;
    while (millis() < DriptingTime) {
      if (millis() >= DriptingTime) {
        Defrosting = 0;
        doorCount = 0;
        break;
      }
    }
  }
}

// *********************************************************************
// CompressorStart
// *********************************************************************
void CompressorStart()
{
  if (FreezeSensor >= (FreezeSet + 3)) {
    digitalWrite(Compressor, HIGH);
    digitalWrite(FanMotor, HIGH);
  }
}

// *********************************************************************
// CompressorStop
// *********************************************************************
void CompressorStop()
{
  if (FreezeSensor <= (FreezeSet - 2)) {
    digitalWrite(Compressor, LOW);
    digitalWrite(FanMotor, LOW);
    StartDelay = millis() + 300000;
  }
}

// *********************************************************************
// GetFreezeTemp
// *********************************************************************
void GetFreezeTemp()
{
  int RawADC;
  RawADC = analogRead(A0);
  double Temp;
  Temp = log(10000.0 * ((1024.0 / RawADC - 1)));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp );
  FreezeSensor = Temp - 273.15;
  ADCFilter.Filter(FreezeSensor);
}

// *********************************************************************
// GetEvapSensor
// *********************************************************************
void GetEvapSensor()
{
  int RawADC;
  RawADC = analogRead(A1);
  double Temp;
  Temp = log(10000.0 * ((1024.0 / RawADC - 1)));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp );
  EvapSensor = Temp - 273.15;
  ADCFilter.Filter(EvapSensor);
}

// *********************************************************************
// GetFridgeSensor
// *********************************************************************
void GetFridgeSensor()
{
  int RawADC;
  RawADC = analogRead(A2);
  double Temp;
  Temp = log(10000.0 * ((1024.0 / RawADC - 1)));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp );
  FridgeSensor = Temp - 273.15;
  ADCFilter.Filter(FridgeSensor);
}
