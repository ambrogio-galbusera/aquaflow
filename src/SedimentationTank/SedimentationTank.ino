/**********************************************************
 * Sedimentation Tank
 */
#include "Ultrasonic.h"

// Pin definitions
#define PIN_ULTRASONIC  0
#define PIN_PUMP        3
#define PIN_COMPRESSOR  2
#define PIN_LDR         1

// Minimum water level
#define CM_MIN_THRESH   10
// Water half level
#define CM_HALF_THRESH  (TANK_HEIGHT/2)
// Maximum water level
#define CM_MAX_THRESH   (TANK_HEIGHT*9/10)
// Height hysteresis
#define CM_HYSTERESIS   3
// Filter weight
#define CM_ALPHA        0.1f

// Water level calculation parameters
// Sensor offset above the tank cover
#define SENSOR_OFFSET        8
// Tank height
#define TANK_HEIGHT          200
// Convert sonar reading to water level
#define CM_TO_HEIGHT(a)      ( ((a)<TANK_HEIGHT+SENSOR_OFFSET)? (TANK_HEIGHT-(a-SENSOR_OFFSET)) : 0 )

// Compressor ON time
#define COMPRESSOR_ON_MS    (10 * 1000)
// Compressor OFF time
#define COMPRESSOR_OFF_MS   (10 * 60 * 1000)
// Compressor total cycle time
#define COMPRESSOR_CYCLE_MS (2 * 60 * 60 * 1000)

// Pump ON time
#define PUMP_ON_MS          (10 * 1000)
// Pump OFF fime
#define PUMP_OFF_MS         (10 * 1000)

// Ultrasonic sensor object
Ultrasonic ultrasonic(PIN_ULTRASONIC);

// Current and previous sonar reading
long prevCm = CM_MAX_THRESH;
long currCm = 0;

// LDR previous status
int prevLDR;

// Compressor status variables
bool compressorOn;
unsigned long compressorCycleStart;
unsigned long compressorStart;

// Pump status variables
bool pumpOn;
unsigned long pumpStart;
long pumpExpCm; 

////////////////////////////////////////////////////////////////
// Switch compressor on/off
void setCompressor(bool on)
{
  Serial.print("Setting compressor ");
  Serial.println(on? "HIGH" : "LOW");

  digitalWrite(PIN_COMPRESSOR, on? HIGH : LOW);
  compressorOn = on;  
  compressorStart = millis();
}

////////////////////////////////////////////////////////////////
// Switch pump on/off
void setPump(bool on)
{
  Serial.print("Setting pump ");
  Serial.println(on? "HIGH" : "LOW");

  digitalWrite(PIN_PUMP, on? HIGH : LOW);
  pumpOn = on;  
  pumpStart = millis();
}

////////////////////////////////////////////////////////////////
// Apply a moving average to the sonar reading and store 
// filtered value in global var currCm
long filterCm(long cm)
{
  float tmp = currCm;
  float tmp1 = cm;
  float tmp2 = (tmp * (1-CM_ALPHA)) + (tmp1 * CM_ALPHA);

  prevCm = currCm;
  currCm = (long)tmp2;
}

////////////////////////////////////////////////////////////////
// Sketch setup function
void setup() {
  pinMode(PIN_PUMP, OUTPUT);
  setPump(false);
  
  pinMode(PIN_COMPRESSOR, OUTPUT);
  setCompressor(false);

  pinMode(PIN_LDR, INPUT);
  prevLDR = digitalRead(PIN_LDR);
}

////////////////////////////////////////////////////////////////
// Sketch main loop
void loop() {
  unsigned long delta;
  long rd, cm;
  
  if (compressorCycleStart != 0)
  {
    // compressor cycle is active
    if (compressorOn)
    {
      // compressor is on, check if it is time to switch it off
      delta = millis() - compressorStart;
      if (delta > COMPRESSOR_ON_MS)
      {
        setCompressor(false);

        // check if the compressor cycle duration has ben reached
        delta = millis() - compressorCycleStart;
        if (delta > COMPRESSOR_CYCLE_MS)
        {
          compressorCycleStart = 0;
        }
      }
    }
    else
    {
      // compressor is off, check if it is time to switch it on
      unsigned long delta = millis() - compressorStart;
      if (delta > COMPRESSOR_OFF_MS)
      {
        setCompressor(true);
      }
    }
  }

  if (pumpOn)
  {
    // pump is on. Keep pump on for a maximum of PUMP_ON_MS, then switch it off
    delta = millis() - pumpStart;
    if (delta > PUMP_ON_MS)
    {
      setPump(false);
    }
  }
  else
  {
    // pump is off. If the pumpExpCm is set, then we need to check if it's time 
    // to sitch the pump on
    if (pumpExpCm != 0)
    {
      delta = millis() - pumpStart;
      if (delta > PUMP_OFF_MS)
      {
        setPump(true);
      }
    }
  }

  // update water level
  rd = ultrasonic.MeasureInCentimeters();
  cm = CM_TO_HEIGHT(rd);
  filterCm(cm);
  Serial.print("Cm: ");
  Serial.print(cm);
  Serial.print(",");
  Serial.print(rd);
  Serial.print(", Filtered: ");
  Serial.print(currCm);
  Serial.print(" / ");
  Serial.print(prevCm);
  Serial.print(" / ");
  Serial.println(pumpExpCm);

  // check if the expected level has been reached
  if (pumpExpCm != 0)
  {
    if (currCm < pumpExpCm)
    {
      // expected level reached: switch off pump
      Serial.print("Level reached ");
      Serial.print(currCm);
      Serial.print(" / ");
      Serial.println(pumpExpCm);

      setPump(false);
      pumpExpCm = 0; 
    }
  }
  
  if (currCm > prevCm + CM_HYSTERESIS)
  {
    // water added. Start compressor cycle
    if (compressorCycleStart == 0) {
      setCompressor(true);
    }
    
    compressorCycleStart = millis();
  }

  // update LDR
  int ldr = digitalRead(PIN_LDR);
  Serial.print("LDR: ");
  Serial.println(ldr);

  if ((ldr == 1) && (prevLDR == 0))
  {
    // new day: switch on pump until tank is empty
    Serial.println("New day!");

    pumpExpCm = CM_MIN_THRESH;
    if (!pumpOn)
      setPump(true);

    prevLDR = ldr;
  }

  if ((currCm > CM_MAX_THRESH) && (pumpExpCm == 0))
  {
    // tank is full: pump on
    Serial.print("Level > max ");
    Serial.print(currCm);
    Serial.print(" / ");
    Serial.println(CM_MAX_THRESH);

    pumpExpCm = CM_HALF_THRESH;
    if (!pumpOn)
      setPump(true);
  }

  if (currCm < CM_MIN_THRESH)
  {
    // tank is empty: pump off
    Serial.print("Level < min ");
    Serial.print(currCm);
    Serial.print(" / ");
    Serial.println(CM_MIN_THRESH);
    
    pumpExpCm = 0;
    setPump(false);
  }

  delay(1000);
}
