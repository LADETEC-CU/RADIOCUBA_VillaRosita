#include <Arduino.h>
#include <Wire.h>
#include <AHTxx.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
# include <ModbusRTUSlave.h>
#include <EEPROM.h>

boolean debugSerial               = true;
boolean sensorTempAHT1            = false;
boolean tcSimulation              = false;           // false - real,  true - simulation
boolean tempAutocompensation      = false;

const uint8_t pinLEDbuiltin       = 2;  // LED built-in, general status, flashing in nomal working
const uint8_t dePin               = 27; // Serial1 RS-485 control
const uint8_t pinHumSensor        = 15; // Data from DTH temperture-humidity sensor

const uint8_t pinAC1control       = 32; // (26) DO0 on/off air conditioner 1
const uint8_t pinAC2control       = 33; // (25) DO1 on/off air conditioner 2
const uint8_t pinDO2              = 25; // DO2
const uint8_t pinDO3              = 26; // DO3

const uint8_t pinAC1status        =  4; // (18) monitor air conditioner 1
const uint8_t pinAC2status        =  5; // (19) monitor air conditioner 2
const uint8_t pinAlarmArmed       = 18; // ( 5) NO FUNCIONO, alarm: 0 -disarmed, 1 - armed
const uint8_t pinAlarmSiren       = 19; // ( 4) alarm type: 0 - burglary, 1 - fire
const uint8_t pinEGstatus         = 23; // (23) status monitor
const uint8_t pinDI5              = 13; // DI5
const uint8_t pinDI6              = 14; // DI5

const uint8_t pinAI0TempPT100     = 35; // (14) temperature sensor PT-100 (analog)
const uint8_t pinAI1currentAC1    = 34; // AI2
const uint8_t pinAI2currentAC2    = 39; // AI3
const uint8_t pinAI3EGfuelLevel   = 36; // (36) fuel level (analog)

const uint16_t stateOFF           = 0;
const uint16_t stateDesiredOn     = 1;
const uint16_t stateON            = 2;

float Ts                          = 2; // sample time T & H & fuelLevel filters [seconds];
ulong timeOn                      = 1000;
ulong timeOff                     = 2000;
boolean ledOn                     = false;

ModbusRTUSlave modbus(Serial2, dePin); 
uint8_t slaveId = 10;
uint32_t baud = 19200;

const uint16_t numInputRegisters = 12;
const uint16_t numHoldingRegisters = 16;
const uint16_t numDiscreteInputs = 7;
const uint16_t numDiscreteCoils = 8;

uint16_t inputRegisters[numInputRegisters];
uint16_t holdingRegisters[numHoldingRegisters];
boolean discreteInputs[numDiscreteInputs];
boolean discreteCoils[numDiscreteCoils] = {true, true, true, true};

// AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR);   // sensor address, sensor type
boolean firstSensorRead = true;
boolean readSensor = false;

DHT_Unified dht(pinHumSensor, DHT11);

float temperature;                              // to store T/RH result
int16_t temperatureCorrection;                  // temperature correction
uint16_t tempPT100;
int16_t tempAutocompensationValue;
float fT_filtered;
float fT_k_1 = 27;
uint16_t fT_T = 30;                             // time constant Temperature filter [seconds];

float humedity;                                 // to store T/RH result
float fH_filtered;
float fH_k_1 = 27;
uint16_t fH_T = 30;                             // time constant Humedity filter [seconds];

float currentAC1;                               // to store T/RH result
uint16_t currrentAC1mx = 1000;
float fcAC1_filtered;
float fcAC1_k_1;
uint16_t fcAC1_T = 30;                          // time constant Humedity filter [seconds];

float currentAC2;                               // to store T/RH result
uint16_t currrentAC2mx = 1000;
float fcAC2_filtered;
float fcAC2_k_1;
uint16_t fcAC2_T = 30;                          // time constant Humedity filter [seconds];

uint16_t fuelLevelEG;
float fFL_filtered;
float fFL_k_1 = 50;
uint16_t fFL_T = 10;                            // time constant Fuel Level filter [seconds];

// Data in EEPROM
uint16_t tcSetPoint               = 200;         // degree Celcius
uint16_t tcDelta                  = 20;          // degree Celcius
boolean  tcMode                   = true;        // 0 - manual,  1 - automatic
uint16_t tcControlState           = 0;           // -1 under, 0 between, 1 over
boolean  pAirRotation             = true;        // 0 - no rotation,  1 - rotation
uint16_t pProtectionTime          = 21; //3 * 60 * 1000;     // miliseconds
uint16_t pAirRotationTime         = 100; //4 * 3600;   // seconds
uint16_t pAirToTurnOff            = 1;

uint16_t tcSimulationTimeConstant = 50; //20 * 60;    // seconds;
uint16_t tcSimulationAirGain1     = 7;          // degree Celcius air 1
uint16_t tcSimulationAirGain2     = 7;          // degree Celcius air 2

float    tcTsim1;
float    tcTsim2;
float    tcTsim1k_1;
float    tcTsim2k_1;
float    tcT;

// Air protection and rotation
ulong    pNextRotationTime;
ulong    pAir1nextOnTime;
ulong    pAir2nextOnTime;
boolean  pAir1ReadyToOn           = false;
boolean  pAir2ReadyToOn           = false;
boolean  pAir1DesiredState        = false;  
boolean  pAir2DesiredState        = false;
boolean  pAreTwoAirWorking        = false;
uint16_t air1State                = stateOFF;
uint16_t air2State                = stateOFF;

boolean  isEGworking              = false;
boolean  isAA1working             = false;
boolean  isAA2working             = false;
boolean  isAlarmArmed             = false;
boolean  isAlarmSirenON           = false;
boolean  isAlarmFire              = false;
boolean  coilAA1                  = false;
boolean  coilAA2                  = false;
boolean  coilDO2                  = false;
boolean  coilDO3                  = false;

// Onlu useful for test stage
int8_t  count                     = 255;
ulong   runTimeInit               = 0;

// put function declarations here:
void printStatus();

void setup() {

  pinMode(pinAC1control, OUTPUT);
  pinMode(pinAC2control, OUTPUT);
  pinMode(pinDO2, OUTPUT);
  pinMode(pinDO3, OUTPUT);
  
  digitalWrite(pinAC1control, coilAA1);
  digitalWrite(pinAC2control, coilAA2);
  digitalWrite(pinDO2, coilDO2);
  digitalWrite(pinDO3, coilDO3);
  

  if (debugSerial) Serial.begin(baud);
  
  pinMode(pinLEDbuiltin, OUTPUT);
  
  pinMode(pinAI3EGfuelLevel, ANALOG);
  pinMode(pinAI0TempPT100, ANALOG);
  
  pinMode(pinEGstatus, INPUT_PULLUP);
  pinMode(pinAC1status, INPUT_PULLUP);
  pinMode(pinAC2status, INPUT_PULLUP);
  pinMode(pinAlarmArmed, INPUT_PULLUP);
  pinMode(pinAlarmSiren, INPUT_PULLUP);

  modbus.configureDiscreteInputs(discreteInputs, numDiscreteInputs);
  modbus.configureCoils(discreteCoils, numDiscreteCoils);
  modbus.configureInputRegisters(inputRegisters, numInputRegisters);
  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  modbus.begin(slaveId, baud, SERIAL_8N1);

  EEPROM.begin(64);
  
  uint8_t eep0 = EEPROM.read(0);
  uint8_t eep1 = EEPROM.read(1);
  if (debugSerial) {
    Serial.println(eep0, HEX);
    Serial.println(eep1, HEX);
  }
  if (eep0 == 0x33 && eep1 == 0xAA) {
    EEPROM.get(2,  tcSetPoint);
    EEPROM.get(4,  tcDelta);
    EEPROM.get(6,  tcMode);
    EEPROM.get(7,  pAirRotation);
    EEPROM.get(8,  pProtectionTime);
    EEPROM.get(10, pAirRotationTime);
    EEPROM.get(12, fT_T);
    EEPROM.get(14, fH_T);
    EEPROM.get(16, fFL_T);
    EEPROM.get(18, fcAC1_T);
    EEPROM.get(20, fcAC2_T);
    EEPROM.get(22, temperatureCorrection);
    EEPROM.get(24, currrentAC1mx);
    EEPROM.get(26, currrentAC2mx);
    if (debugSerial) Serial.println("-----   eeprom ok ---------");
  } else {
    EEPROM.write(0, 0x33);
    EEPROM.write(1, 0xAA);
    EEPROM.put(2,  tcSetPoint);
    EEPROM.put(4,  tcDelta);
    EEPROM.put(6,  tcMode);
    EEPROM.put(7,  pAirRotation);
    EEPROM.put(8,  pProtectionTime);
    EEPROM.put(10, pAirRotationTime);
    EEPROM.put(12, fT_T);
    EEPROM.put(14, fH_T);
    EEPROM.put(16, fFL_T);
    EEPROM.put(18, fcAC1_T);
    EEPROM.put(20, fcAC2_T);
    EEPROM.put(22, temperatureCorrection);
    EEPROM.put(24, currrentAC1mx);
    EEPROM.put(26, currrentAC2mx);
    EEPROM.commit();
    if (debugSerial) Serial.println("********   EEPROM inicializada *********");
  }
  pAir1nextOnTime = pProtectionTime * 1000;
  pAir2nextOnTime = pProtectionTime * 1000 + 20000;
  pNextRotationTime = pAirRotationTime * 1000;

  // if (sensorTempAHT1) {
  //   while (aht10.begin() != true) //for ESP-01 use aht10.begin(0, 2)
  //   {
  //     if (debugSerial) Serial.println(F("AHT1x not connected or fail to load calibration coefficient"));
  //     delay(1000);
  //   }
  //   if (debugSerial) Serial.println(F("AHT10 OK"));
  // }

  dht.begin();
  
} // setup

void loop() {
  
  // timer to read sensor and manage built-in LED
  ulong runTime = millis();
  if ((runTime > timeOn) && !ledOn) {
    digitalWrite(pinLEDbuiltin, HIGH);
    ledOn = true;
    readSensor = true;
  }
  if ((runTime > timeOff) && ledOn) {
    digitalWrite(pinLEDbuiltin, LOW);
    timeOn += 2000;   // time ON
    timeOff += 2000;  // time OFF
    ledOn = false;
  }

  // sensors reading and filtering
  if (readSensor) {
  
    // Get temperature
    if (tempAutocompensation) tempAutocompensationValue = - 0.5 * (coilAA1 ? 1 : 0) - 0.5 * (coilAA2 ? 1 : 0);
    else tempAutocompensationValue = 0;
    tempPT100 = analogRead(pinAI0TempPT100);
    temperature = tempPT100 * 0.04281 - 1.768 + tempAutocompensationValue +  (temperatureCorrection / 100.0);
    
    // Get humidity event and its value.
    sensors_event_t event;
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      humedity = 8;
    } else {
      humedity = event.relative_humidity;
    }

    // get current AC1
    currentAC1 = analogRead(pinAI1currentAC1) * currrentAC1mx / 1000.0;

    // get current AC2
    currentAC2 = analogRead(pinAI2currentAC2) * currrentAC2mx / 1000.0;

    // get fuel level
    fuelLevelEG = analogRead(pinAI3EGfuelLevel);
    
    
    if (firstSensorRead) {
      firstSensorRead = false;
      fT_k_1 = temperature;   // initialization of Temperature and Humedity filters
      fH_k_1 = humedity;
      fcAC1_k_1 = currentAC1;
      fcAC2_k_1 = currentAC2;
      fFL_k_1 = fuelLevelEG;
    }
    fT_filtered = (1-exp(-Ts / ((float) fT_T))) * temperature + exp(-Ts / ((float) fT_T)) * fT_k_1;
    fH_filtered = (1-exp(-Ts / ((float) fH_T))) * humedity + exp(-Ts / ((float) fH_T))* fH_k_1;
    fcAC1_filtered = (1-exp(-Ts / ((float) fcAC1_T))) * currentAC1 + exp(-Ts / ((float) fcAC1_T))* fcAC1_k_1;
    fcAC2_filtered = (1-exp(-Ts / ((float) fcAC2_T))) * currentAC2 + exp(-Ts / ((float) fcAC2_T))* fcAC2_k_1;
    fFL_filtered = (1-exp(-Ts / ((float) fFL_T))) * fuelLevelEG + exp(-Ts / ((float) fFL_T)) * fFL_k_1;
    
    fT_k_1 = fT_filtered;
    fH_k_1 = fH_filtered;
    fcAC1_k_1 = fcAC1_filtered;
    fcAC2_k_1 = fcAC2_filtered;
    fFL_k_1 = fFL_filtered;

    // Temperature simulation
    if (tcSimulation) {
      tcTsim1k_1 = tcTsim1;
      tcTsim1 = (coilAA1 ? 1 : 0) * tcSimulationAirGain1 * (1-exp(-Ts / ((float) tcSimulationTimeConstant))) + exp(-Ts / ((float) tcSimulationTimeConstant))* tcTsim1k_1;
      tcTsim2k_1 = tcTsim2;
      tcTsim2 = (coilAA2 ? 1 : 0) * tcSimulationAirGain2 * (1-exp(-Ts / ((float) tcSimulationTimeConstant))) + exp(-Ts / ((float) tcSimulationTimeConstant))* tcTsim2k_1;
      tcT = 30 - tcTsim1 - tcTsim2;
    } else {
      tcT = fT_filtered;
      tcTsim1k_1 = 0;
      tcTsim2k_1 = 0;
    }
  }

  uint16_t mxRotation = 1;

  // Temperature control
  if (tcMode) {
    if (tcT < (tcSetPoint - tcDelta) / 10.0) {
      if (pAreTwoAirWorking) {
        if (pAirToTurnOff == 1) {
          pAir1DesiredState = false;
          if (pAirRotation) pAirToTurnOff = 2;
        } else {
          pAir2DesiredState = false;
          if (pAirRotation) pAirToTurnOff = 1;
        }
        pAreTwoAirWorking = false;
        pNextRotationTime = runTime + pAirRotationTime * 1000;
      }
    }
    if (tcT > (tcSetPoint + tcDelta) / 10.0) {
      pAir1DesiredState = true;
      pAir2DesiredState = true;
      pAreTwoAirWorking = true;
    }

    // Air rotation
    if (!pAreTwoAirWorking && pAirRotation) {
      if (runTime > pNextRotationTime) {
        mxRotation = 5;
        if (pAirToTurnOff == 1) {
          pAir1DesiredState = false;
          pAir2DesiredState = true; 
          pAirToTurnOff = 2;
        } else {
          pAir1DesiredState = true; 
          pAir2DesiredState = false;
          pAirToTurnOff = 1;
        }
        pNextRotationTime = runTime + pAirRotationTime * 1000;
      }
    } else pNextRotationTime = 0;
  }

  pAir1ReadyToOn = (runTime > pAir1nextOnTime) ? true : false; 
  pAir2ReadyToOn = (runTime > pAir2nextOnTime) ? true : false; 
  
  // Air 1 protection
  switch (air1State) {
    case stateOFF:
      if (pAir1DesiredState) {
        air1State = stateDesiredOn;
      }
      coilAA1 = 0;
      break;
    case stateDesiredOn:
      if (pAir1ReadyToOn) {
        coilAA1 = 1;
        air1State = stateON;
      }
      break;
    case stateON:
      coilAA1 = 1;
      if (!pAir1DesiredState) {
        air1State = stateOFF;
        pAir1nextOnTime = runTime + mxRotation * pProtectionTime * 1000;
      }
      break;
    default:
      air1State = stateOFF;
      break;
  }

  // Air 2 protection
  switch (air2State) {
    case stateOFF:
      if (pAir2DesiredState) {
        air2State = stateDesiredOn;
      }
      coilAA2 = 0;
      break;
    case stateDesiredOn:
      if (pAir2ReadyToOn) {
        coilAA2 = 1;
        air2State = stateON;
      }
      break;
    case stateON:
      coilAA2 = 1;
      if (!pAir2DesiredState) {
        air2State = stateOFF;
        pAir2nextOnTime = runTime + mxRotation * pProtectionTime * 1000;
      }
      break;
    default:
      air2State = stateOFF;
      break;
  }

  // Modbus

  uint16_t tcSetPointLast               = tcSetPoint;
  uint16_t tcDeltalast                  = tcDelta;
  boolean  tcModeLast                   = tcMode;
  boolean  pAirRotationLast             = pAirRotation;
  uint16_t pProtectionTimeLast          = pProtectionTime;
  uint16_t pAirRotationTimeLast         = pAirRotationTime;
  uint16_t pAirToTurnOffLast            = pAirToTurnOff;
  uint16_t fT_TLast                     = fT_T;
  uint16_t fH_TLast                     = fH_T;
  uint16_t fFL_TLast                    = fFL_T;
  uint16_t fcAC1_TLast                  = fcAC1_T;
  uint16_t fcAC2_TLast                  = fcAC2_T;
  int16_t  temperatureCorrectionLast    = temperatureCorrection;
  uint16_t currentAC1mxLast             = currrentAC1mx;
  uint16_t currentAC2mxLast             = currrentAC2mx;

  isAA1working = digitalRead(pinAC1status);
  isAA2working = digitalRead(pinAC2status);
  isAlarmArmed = digitalRead(pinAlarmArmed);
  isAlarmSirenON = digitalRead(pinAlarmSiren);
  isEGworking = digitalRead(pinEGstatus);

  discreteInputs[0] = isAA1working;
  discreteInputs[1] = isAA2working;
  discreteInputs[2] = isAlarmArmed;
  discreteInputs[3] = isAlarmSirenON;
  discreteInputs[4] = isEGworking;
  discreteInputs[5] = digitalRead(pinDI5);
  discreteInputs[6] = digitalRead(pinDI6);

  inputRegisters[ 0] = (uint16_t) (temperature * 100);
  inputRegisters[ 1] = (uint16_t) (humedity * 100);
  inputRegisters[ 2] = fuelLevelEG;
  inputRegisters[ 3] = millis() % 65536;
  inputRegisters[ 4] = (uint16_t) (tcT * 100);
  inputRegisters[ 5] = (uint16_t) (fH_filtered * 100);
  inputRegisters[ 6] = (uint16_t) (round(fFL_filtered));
  inputRegisters[ 7] = ((int64_t) (pAir1nextOnTime) - runTime) > 0 ? (((int64_t) (pAir1nextOnTime) - runTime) / 1000.0) : 0;
  inputRegisters[ 8] = ((int64_t) (pAir2nextOnTime) - runTime) > 0 ? ((int64_t) (pAir2nextOnTime) - runTime) / 1000.0 : 0;
  inputRegisters[ 9] = ((int64_t) (pNextRotationTime) - runTime) > 0 ? ((int64_t) (pNextRotationTime) - runTime) / 1000.0 : 0;
  inputRegisters[10] = (uint16_t) (fcAC1_filtered * 10);
  inputRegisters[11] = (uint16_t) (fcAC2_filtered * 10);
  
  if (!tcMode) {
    discreteCoils[0] = pAir1DesiredState;
    discreteCoils[1] = pAir2DesiredState;
  } else {
    discreteCoils[0] = coilAA1;
    discreteCoils[1] = coilAA2;
  }
  discreteCoils[2] = coilDO2;
  discreteCoils[3] = coilDO3;
  discreteCoils[4] = tcSimulation;
  discreteCoils[5] = tcMode;
  discreteCoils[6] = pAirRotation;
  discreteCoils[7] = tempAutocompensation;

  holdingRegisters[ 0] = fT_T;
  holdingRegisters[ 1] = fH_T;
  holdingRegisters[ 2] = fFL_T;
  holdingRegisters[ 3] = tcSetPoint;
  holdingRegisters[ 4] = tcDelta;
  holdingRegisters[ 5] = pProtectionTime;
  holdingRegisters[ 6] = pAirRotationTime;
  holdingRegisters[ 7] = tcSimulationTimeConstant;
  holdingRegisters[ 8] = tcSimulationAirGain1;
  holdingRegisters[ 9] = tcSimulationAirGain2;
  holdingRegisters[10] = pAirToTurnOff;
  holdingRegisters[11] = temperatureCorrection;
  holdingRegisters[12] = fcAC1_T;
  holdingRegisters[13] = fcAC2_T;
  holdingRegisters[14] = currrentAC1mx;
  holdingRegisters[15] = currrentAC2mx;
  
  modbus.poll();

  coilDO2 = discreteCoils[2];
  coilDO3 = discreteCoils[3];
  tcSimulation = discreteCoils[4];
  tcMode = discreteCoils[5];
  pAirRotation = discreteCoils[6];
  tempAutocompensation = discreteCoils[7];
  
  if (!tcMode) {
    pAir1DesiredState = discreteCoils[0];
    pAir2DesiredState = discreteCoils[1];
  } else {
    coilAA1 = discreteCoils[0];
    coilAA2 = discreteCoils[1];
  }

  fT_T = holdingRegisters[0];
  fH_T = holdingRegisters[1];
  fFL_T = holdingRegisters[2];
  tcSetPoint = holdingRegisters[3];
  if (tcSetPoint < 150) tcSetPoint = 150;
  if (tcSetPoint > 300) tcSetPoint = 300;
  tcDelta = holdingRegisters[4];
  if (tcDelta < 10) tcDelta = 10;
  if (tcDelta > 50) tcDelta = 50;
  pProtectionTime = holdingRegisters[5];
  pAirRotationTime = holdingRegisters[6];
  tcSimulationTimeConstant = holdingRegisters[7];
  tcSimulationAirGain1 = holdingRegisters[8];
  tcSimulationAirGain2 = holdingRegisters[9];
  temperatureCorrection = holdingRegisters[11];
  fcAC1_T = holdingRegisters[12];
  fcAC1_T = holdingRegisters[13];
  currrentAC1mx = holdingRegisters[14];
  currrentAC2mx = holdingRegisters[15];

  
  if (tcSetPoint != tcSetPointLast) {
    EEPROM.put(2, tcSetPoint);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado Set Point");
  }
  if (tcDelta != tcDeltalast) {
    EEPROM.put(4, tcDelta);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado Delta");
  }
  if (tcMode != tcModeLast) {
    EEPROM.put(6, tcMode);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado Modo");
    if (tcMode && air1State == 0 && air2State == 0) {
      pAir2nextOnTime = runTime + 20000;
    }
  }
  if (pAirRotation != pAirRotationLast) {
    EEPROM.put(7, pAirRotation);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado Air Rotation");
  }
  if (pProtectionTime != pProtectionTimeLast) {
    EEPROM.put(8, pProtectionTime);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado Protection Time");
  }
  if (pAirRotationTime != pAirRotationTimeLast) {
    EEPROM.put(10, pAirRotationTime);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado Rotation Time");
  }
  if (fT_T != fT_TLast) {
    EEPROM.put(12, fT_T);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado Temperature filter time constant");
  }
  if (fH_T != fH_TLast) {
    EEPROM.put(14, fH_T);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado Humedity filter time constant");
  }
  if (fFL_T != fFL_TLast) {
    EEPROM.put(16, fFL_T);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado Fuel Level filter time constant");
  }
  if (fcAC1_T != fcAC1_TLast) {
    EEPROM.put(18, fcAC1_T);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado Current AC1 filter time constant");
  }
  if (fcAC2_T != fcAC2_TLast) {
    EEPROM.put(20, fcAC2_T);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado Current AC2 filter time constant");
  }
  if (temperatureCorrection != temperatureCorrectionLast) {
    EEPROM.put(22, temperatureCorrection);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado temperatureCorrection");
  }
  if (currrentAC1mx != currentAC1mxLast) {
    EEPROM.put(24, currrentAC1mx);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado mx current AC1");
  }
  if (currrentAC2mx != currentAC2mxLast) {
    EEPROM.put(26, currrentAC2mx);
    EEPROM.commit();
    if (debugSerial) Serial.println ("actualizado mx current AC2");
  }
  
  digitalWrite(pinAC1control, coilAA1);
  digitalWrite(pinAC2control, coilAA2);
  digitalWrite(pinDO2, coilDO2);
  digitalWrite(pinDO3, coilDO3);
  

  // --- debug ------------------------------------------------------------------------------
  if (debugSerial && readSensor) {
    Serial.print(F("DI: "));
    Serial.print(isAA1working);
    Serial.print(F(" "));
    Serial.print(isAA2working);
    Serial.print(F(" "));
    Serial.print(isAlarmArmed);
    Serial.print(F(" "));
    Serial.print(isAlarmSirenON);
    Serial.print(F(" "));
    Serial.print(isEGworking);
    
    Serial.print(F("  AI: T:"));
    Serial.print(tempPT100);
    Serial.print(F(","));
    Serial.print(temperature);
    Serial.print(F(","));
    Serial.print(fT_filtered);
    Serial.print(F(" \tH:"));
    Serial.print(humedity);
    Serial.print(F(","));
    Serial.print(fH_filtered);
    Serial.print(F("  \tc1:"));
    Serial.print(currentAC1);
    Serial.print(F(","));
    Serial.print(fcAC1_filtered);
    Serial.print(F(" \tc2:"));
    Serial.print(currentAC2);
    Serial.print(F(","));
    Serial.print(fcAC2_filtered);
    Serial.print(F(" \tFL:"));
    Serial.print(fuelLevelEG);
    Serial.print(F(","));
    Serial.print(fFL_filtered);
    
    Serial.println("");
  }

  readSensor = false;
} // loop