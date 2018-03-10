/*
  Управление за хладилник модел: rl33sbsw
  от Тони Стоянов
*/
#include <math.h>
#include "MegunoLink.h"
#include "Filter.h"

// Create a new exponential filter with a weight of 20 and initial value of 0.
ExponentialFilter<long> ADCFilter(20, 0);

//Дефинираме входовете
float FreezeSensor;
float EvapSensor;
float FridgeSensor;
bool DoorSwitch;
bool GearMSwitch;

//Дефинираме изходите
int DefrostHeater = 4;
int Compressor = 3;
int FanMotor = 5;
int InsideLamp = 6;
int GearMotor = 7;
int Led_1_2_3_Anode = 8;
int Led_4_5_Anode = 9;
int Led_3_4_Cathode = 10;
int Led_2_5_Cathode = 11;
int Led_1_Cathode = 12;
int ComButtons = 13;

//Дефинираме променливите
int FreezeSet = -20;
int FridgeSet = 3;
int doorCount;
bool Defrosting = 0;
bool FirstDefrost = 0;
byte DefTimeCount;
byte DefTimeSet;
bool Moving;
float RawADC;
// resistance at 25 degrees C
#define ThermistorNominal 5000
// temp. for nominal resistance (almost always 25 C)
#define TemperatureNominal 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCoefficient 3950
// the value of the 'other' resistor
#define SeriesResistor 10000


unsigned long StartDelay;
unsigned long DefrostInterval;
unsigned long DefrostTime;
unsigned long DriptingTime;
unsigned long DoorAlarm;

void setup() {
  Serial.begin(115000);
  Serial.println("Setup");

  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(DefrostHeater, OUTPUT);
  pinMode(Compressor, OUTPUT);
  pinMode(FanMotor, OUTPUT);
  pinMode(InsideLamp, OUTPUT);
  pinMode(GearMotor, OUTPUT);
  pinMode(Led_1_2_3_Anode, OUTPUT);
  pinMode(Led_4_5_Anode, OUTPUT);
  pinMode(Led_3_4_Cathode, OUTPUT);
  pinMode(Led_2_5_Cathode, OUTPUT);
  pinMode(Led_1_Cathode, OUTPUT);

  digitalWrite(DefrostHeater, LOW);
  digitalWrite(Compressor, LOW);
  digitalWrite(FanMotor, LOW);
  digitalWrite(InsideLamp, HIGH);
  digitalWrite(GearMotor, LOW);

  StartDelay = millis() + 120000;
  DefrostInterval = millis + 14400000;
  DefTimeSet = 1;
  digitalWrite(GearMotor, HIGH);
  delay(1000);
  
  digitalWrite(Led_1_2_3_Anode, HIGH);
  digitalWrite(Led_4_5_Anode, HIGH);
  digitalWrite(Led_3_4_Cathode, LOW);
  digitalWrite(Led_2_5_Cathode, LOW);
  digitalWrite(Led_1_Cathode, LOW);
  
  while (digitalRead(A4) == HIGH) {
    digitalWrite(GearMotor, HIGH);
    if (digitalRead(A4) == LOW) {
      delay(500);
      digitalWrite(GearMotor, LOW);
      Moving = 0;
      break;
    }
  }
}

// *********************************************************************
// loop
// *********************************************************************
void loop() {

  Serial.print("FridgeSensor = ");
  Serial.println(FridgeSensor);
  Serial.print("EvapSensor = ");
  Serial.println(EvapSensor);
  Serial.print("FreezeSensor = ");
  Serial.println(FreezeSensor);
  DoorSwitch = digitalRead(A3);
  GearMSwitch = digitalRead(A4);
  Serial.println(GearMSwitch);

  if (Defrosting == 0 && DefTimeCount >= 1 && DefTimeSet == 0) {
    if (FreezeSensor > 4 || FridgeSensor > 6 || doorCount > 1 || FirstDefrost == 0) {
      DefrostInterval = millis + 21600000;
      DefTimeSet = 1;
    }
  }

  if (FreezeSensor >= (FreezeSet + 2) && millis() > StartDelay && Defrosting == 0) {
    CompressorStart();
  }
  else if (FreezeSensor <= FreezeSet) {
    CompressorStop();
  }

  if (millis() > DefrostInterval) {
    CompressorStop();
    DefrostControl();
  }

  DoorControl();
  GetFreezeTemp();
  GetEvapSensor();
  GetFridgeSensor();
  FridgeTempControl();
  UpdateLeds();
}

// *********************************************************************
// FridgeTempControl
// *********************************************************************
void FridgeTempControl()
{
  if (Defrosting  == 0 && FridgeSensor >= FridgeSet + 2 && DoorSwitch == HIGH && digitalRead(A4) == LOW) {
    digitalWrite(FanMotor, HIGH);
    unsigned long TimeToOpen;
    TimeToOpen = millis() + 3000;
    while (millis() < TimeToOpen) {
      digitalWrite(GearMotor, HIGH);
      if (digitalRead(A3) == HIGH) {
        digitalWrite(InsideLamp, LOW);
      }
      if (millis() >= TimeToOpen) {
        digitalWrite(GearMotor, LOW);
        break;
      }
    }
  }

  if ((FridgeSensor < FridgeSet - 2 || DoorSwitch == LOW || Defrosting == 1) && digitalRead(A4) == HIGH) {
    digitalWrite(FanMotor, LOW);
    while (digitalRead(A4) == HIGH) {
      digitalWrite(GearMotor, HIGH);
      if (digitalRead(A3) == HIGH) {
        digitalWrite(InsideLamp, LOW);
      }
      if (digitalRead(A3) == LOW) {
        digitalWrite(InsideLamp, HIGH);
      }
      if (digitalRead(A4) == LOW) {
        delay(500);
        digitalWrite(GearMotor, LOW);
        digitalWrite(FanMotor, HIGH);
        Moving = 0;
        break;
      }
    }
  }
}


// *********************************************************************
// UpdateLeds
// *********************************************************************
void UpdateLeds()
{
  /*
    Led_1_2_3_Anode
    Led_4_5_Anode
    Led_3_4_Cathode
    Led_2_5_Cathode
    Led_1_Cathode
  */
  if (FreezeSet == -18) {
    digitalWrite(Led_1_2_3_Anode, HIGH);
    digitalWrite(Led_4_5_Anode, LOW);
    digitalWrite(Led_3_4_Cathode, HIGH);
    digitalWrite(Led_2_5_Cathode, LOW);
    digitalWrite(Led_1_Cathode, HIGH);
  }
  if (FreezeSet == -19) {
    digitalWrite(Led_1_2_3_Anode, HIGH);
    digitalWrite(Led_4_5_Anode, LOW);
    digitalWrite(Led_3_4_Cathode, HIGH);
    digitalWrite(Led_2_5_Cathode, LOW);
    digitalWrite(Led_1_Cathode, HIGH);
  }
  if (FreezeSet == -20) {
    digitalWrite(Led_1_2_3_Anode, HIGH);
    digitalWrite(Led_4_5_Anode, LOW);
    digitalWrite(Led_3_4_Cathode, LOW);
    digitalWrite(Led_2_5_Cathode, HIGH);
    digitalWrite(Led_1_Cathode, HIGH);
  }
  if (FreezeSet == -21) {
    digitalWrite(Led_1_2_3_Anode, LOW);
    digitalWrite(Led_4_5_Anode, HIGH);
    digitalWrite(Led_3_4_Cathode, LOW);
    digitalWrite(Led_2_5_Cathode, HIGH);
    digitalWrite(Led_1_Cathode, HIGH);
  }
  if (FreezeSet == -22) {
    digitalWrite(Led_1_2_3_Anode, LOW);
    digitalWrite(Led_4_5_Anode, HIGH);
    digitalWrite(Led_3_4_Cathode, HIGH);
    digitalWrite(Led_2_5_Cathode, LOW);
    digitalWrite(Led_1_Cathode, HIGH);
  }

}
// *********************************************************************
// DoorControl
// *********************************************************************
void DoorControl()
{
  DoorSwitch = digitalRead(A3);
  if (DoorSwitch == LOW) {
    digitalWrite(InsideLamp, HIGH);
    doorCount = doorCount + 1;
    DoorAlarm = millis() + 120000;
  }
  else if (DoorSwitch == HIGH) {
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
    DefrostTime  = millis() + 1800000;
    digitalWrite(DefrostHeater, HIGH);
    digitalWrite(Compressor, LOW);
    digitalWrite(FanMotor, LOW);
  }
  if (EvapSensor >= 12 || millis() > DefrostTime) {
    digitalWrite(DefrostHeater, LOW);
    DriptingTime = millis() + 420000;
    while (millis() < DriptingTime) {
      digitalWrite(GearMotor, HIGH);
      if (millis() >= DriptingTime) {
        Defrosting = 0;
        doorCount = 0;
        FirstDefrost = 1;
        DefrostInterval = millis + 68400000;
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
    DefTimeCount++;
  }
}

// *********************************************************************
// GetFreezeTemp
// *********************************************************************
void GetFreezeTemp()
{
  RawADC = analogRead(A0);
  ADCFilter.Filter(RawADC);
  RawADC = 1023 / RawADC - 1;
  RawADC = SeriesResistor / RawADC;
  float Temp;
  Temp = RawADC / ThermistorNominal;     // (R/Ro)
  Temp = log(Temp);                       // ln(R/Ro)
  Temp /= BCoefficient;                   // 1/B * ln(R/Ro)
  Temp += 1.0 / (TemperatureNominal + 273.15); // + (1/To)
  Temp = 1.0 / Temp;                 // Invert
  Temp -= 273.15;                    // convert to C
  FreezeSensor = Temp;
}

// *********************************************************************
// GetEvapSensor
// *********************************************************************
void GetEvapSensor()
{
  RawADC = analogRead(A1);
  ADCFilter.Filter(RawADC);
  RawADC = RawADC;
  RawADC = 1023 / RawADC - 1;
  RawADC = SeriesResistor / RawADC;
  float Temp;
  Temp = RawADC / ThermistorNominal;     // (R/Ro)
  Temp = log(Temp);                       // ln(R/Ro)
  Temp /= BCoefficient;                   // 1/B * ln(R/Ro)
  Temp += 1.0 / (TemperatureNominal + 273.15); // + (1/To)
  Temp = 1.0 / Temp;                 // Invert
  Temp -= 273.15;                    // convert to C
  EvapSensor = Temp;
}

// *********************************************************************
// GetFridgeSensor
// *********************************************************************
void GetFridgeSensor()
{
  RawADC = analogRead(A2);
  ADCFilter.Filter(RawADC);
  RawADC = RawADC;
  RawADC = 1023 / RawADC - 1;
  RawADC = SeriesResistor / RawADC;
  float Temp;
  Temp = RawADC / ThermistorNominal;     // (R/Ro)
  Temp = log(Temp);                       // ln(R/Ro)
  Temp /= BCoefficient;                   // 1/B * ln(R/Ro)
  Temp += 1.0 / (TemperatureNominal + 273.15); // + (1/To)
  Temp = 1.0 / Temp;                 // Invert
  Temp -= 273.15;                    // convert to C
  FridgeSensor = Temp;
}
