/*
  Управление за хладилник модел: rl33sbsw
  от Тони Стоянов
*/
#include <math.h>
#include "MegunoLink.h"
#include "Filter.h"
#include <EEPROM.h>

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
int Led_4_5_Anode = 10;
int Led_3_4_Cathode_OnOff = 11;
int Led_2_5_Cathode_Select = 12;
int Led_1_Cathode = 13;
int ComButtons = 9;


//Дефинираме променливите
int FreezeSet = -20;
int FridgeSet = 4;
int doorCount = 0;
bool Defrosting = 0;
bool FirstDefrost = 0;
byte DefTimeCount = 0;
byte DefTimeSet = 0;
bool Moving = 0;
float RawADC = 0;
int onOff = 0;
// resistance at 25 degrees C
#define ThermistorNominal 5000
// temp. for nominal resistance (almost always 25 C)
#define TemperatureNominal 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCoefficient 3055
// the value of the 'other' resistor
#define SeriesResistor 10000

int addr = 100;

unsigned long StartDelay;
unsigned long DefrostInterval;
unsigned long DefrostTime;
unsigned long DriptingTime;
unsigned long DoorAlarm;
unsigned long FridgeStart;

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
  pinMode(Led_2_5_Cathode_Select, OUTPUT);
  pinMode(Led_3_4_Cathode_OnOff, OUTPUT);
  pinMode(Led_1_2_3_Anode, OUTPUT);
  pinMode(Led_4_5_Anode, OUTPUT);
  pinMode(Led_1_Cathode, OUTPUT);
  pinMode(ComButtons, OUTPUT);

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
  onOff = EEPROM.read(addr);

  unsigned long Rotation = millis() + 60000;
  while (digitalRead(A4) == HIGH) {
    digitalWrite(GearMotor, HIGH);
    if (digitalRead(A4) == LOW || millis() >= Rotation) {
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
  OnOff();
  DoorControl();
  GetFreezeTemp();
  GetEvapSensor();
  GetFridgeSensor();
  FridgeTempControl();
  Select();
  UpdateLeds();
}

// *********************************************************************
// FridgeTempControl
// *********************************************************************
void FridgeTempControl()
{
  UpdateLeds();
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

  else if ((FridgeSensor < FridgeSet - 0.2 || DoorSwitch == LOW || Defrosting == 1) && digitalRead(A4) == HIGH) {
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
// Select
// *********************************************************************
void Select()
{
  digitalWrite(Led_1_2_3_Anode, LOW);
  digitalWrite(Led_4_5_Anode, LOW);
  digitalWrite(Led_3_4_Cathode_OnOff, LOW);
  digitalWrite(Led_2_5_Cathode_Select, LOW);
  digitalWrite(Led_1_Cathode, LOW);

  pinMode(Led_2_5_Cathode_Select, INPUT_PULLUP);
  digitalWrite(ComButtons, LOW);
  unsigned long push = millis() + 200;
  while (digitalRead(Led_2_5_Cathode_Select) == LOW) {
    // digitalWrite(ComButtons, HIGH);
    Serial.println(digitalRead(Led_2_5_Cathode_Select));
    if (millis() >= push) {
      Serial.println("end");
      FreezeSet = FreezeSet - 1;
      FridgeSet = FridgeSet - 1;
      if (FreezeSet < -22) {
        FreezeSet = -18;
      }
      if (FridgeSet < 1) {
        FridgeSet = 5;
      }

      Serial.println(FreezeSet);
      Serial.println(FridgeSet);
      delay(1000);
      break;
    }
  }
  pinMode(Led_2_5_Cathode_Select, OUTPUT);
  pinMode(Led_3_4_Cathode_OnOff, OUTPUT);
  pinMode(Led_1_2_3_Anode, OUTPUT);
  pinMode(Led_4_5_Anode, OUTPUT);
  pinMode(Led_1_Cathode, OUTPUT);
  pinMode(ComButtons, OUTPUT);
}

// *********************************************************************
// OnOff
// *********************************************************************
void OnOff()
{
  digitalWrite(Led_1_2_3_Anode, LOW);
  digitalWrite(Led_4_5_Anode, LOW);
  digitalWrite(Led_3_4_Cathode_OnOff, LOW);
  digitalWrite(Led_2_5_Cathode_Select, LOW);
  digitalWrite(Led_1_Cathode, LOW);

  pinMode(Led_3_4_Cathode_OnOff, INPUT_PULLUP);
  digitalWrite(ComButtons, LOW);
  unsigned long push = millis() + 500;
  while (digitalRead(Led_3_4_Cathode_OnOff) == LOW) {
    Serial.println(digitalRead(Led_3_4_Cathode_OnOff));
    if (millis() >= push) {
      Serial.println("Off");
      onOff = 1;
      delay(1000);
      EEPROM.write(addr, onOff);
      break;
    }
  }
  while (onOff == 1) {
    digitalWrite(ComButtons, LOW);
    unsigned long push = millis() + 500;
    digitalWrite(DefrostHeater, LOW);
    digitalWrite(Compressor, LOW);
    digitalWrite(FanMotor, LOW);
    digitalWrite(InsideLamp, LOW);
    digitalWrite(GearMotor, LOW);
    while (digitalRead(Led_3_4_Cathode_OnOff) == LOW) {
      if (millis() >= push) {
        Serial.println("On");
        if (onOff == 1) {
          onOff = 0;
          StartDelay = millis() + 60000;
          delay(1000);
          EEPROM.write(addr, onOff);
          break;
        }
      }
    }
  }
  pinMode(Led_2_5_Cathode_Select, OUTPUT);
  pinMode(Led_3_4_Cathode_OnOff, OUTPUT);
  pinMode(Led_1_2_3_Anode, OUTPUT);
  pinMode(Led_4_5_Anode, OUTPUT);
  pinMode(Led_1_Cathode, OUTPUT);
  pinMode(ComButtons, OUTPUT);
}

// *********************************************************************
// UpdateLeds
// *********************************************************************
void UpdateLeds()
{
  if (FreezeSet == -18) {
    digitalWrite(Led_1_2_3_Anode, HIGH);
    digitalWrite(Led_4_5_Anode, LOW);
    digitalWrite(Led_3_4_Cathode_OnOff, LOW);
    digitalWrite(Led_2_5_Cathode_Select, LOW);
    digitalWrite(Led_1_Cathode, HIGH);
  }
  if (FreezeSet == -19) {
    digitalWrite(Led_1_2_3_Anode, HIGH);
    digitalWrite(Led_4_5_Anode, LOW);
    digitalWrite(Led_3_4_Cathode_OnOff, LOW);
    digitalWrite(Led_2_5_Cathode_Select, HIGH);
    digitalWrite(Led_1_Cathode, LOW);
  }
  if (FreezeSet == -20) {
    digitalWrite(Led_1_2_3_Anode, HIGH);
    digitalWrite(Led_4_5_Anode, LOW);
    digitalWrite(Led_3_4_Cathode_OnOff, HIGH);
    digitalWrite(Led_2_5_Cathode_Select, LOW);
    digitalWrite(Led_1_Cathode, LOW);
  }
  if (FreezeSet == -21) {
    digitalWrite(Led_1_2_3_Anode, LOW);
    digitalWrite(Led_4_5_Anode, HIGH);
    digitalWrite(Led_3_4_Cathode_OnOff, HIGH);
    digitalWrite(Led_2_5_Cathode_Select, LOW);
    digitalWrite(Led_1_Cathode, LOW);
  }
  if (FreezeSet == -22) {
    digitalWrite(Led_1_2_3_Anode, LOW);
    digitalWrite(Led_4_5_Anode, HIGH);
    digitalWrite(Led_3_4_Cathode_OnOff, LOW);
    digitalWrite(Led_2_5_Cathode_Select, HIGH);
    digitalWrite(Led_1_Cathode, LOW);
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
    StartDelay = millis() + 360000;
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
