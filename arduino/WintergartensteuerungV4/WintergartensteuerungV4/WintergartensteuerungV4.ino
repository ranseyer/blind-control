/*
 * Changelog: Kommentare zum weiteren Vorgehen eingefügt
 * BME280: neuer Anlauf (BME in 2 Stufen deaktivierbar)
 * Auf Arrays umgebaut
 * Vom Controller aus deaktivierbare Regensensorik => V_VAR1
 * Div. Vorbereitungen für Laufzeit-Berechnungen 
 * Fahrbefehle geändert (schließen = 0, hoch = 100, stop = -1)
 */

#define SN "MultiCover"
#define SV "0.4.1"

//#define MY_DEBUG
//#define MY_DEBUG_LOCAL //Für lokale Debug-Ausgaben
#define MY_DEBUG_ACTUAL //Für lokale Debug-Ausgaben
// Enable RS485 transport layer
#define MY_RS485
//#define MY_RS485_HWSERIAL Serial
// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 2
// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600
#define MY_NODE_ID 112
#define MY_TRANSPORT_WAIT_READY_MS 2000
#include <EEPROM.h>
#include <Arduino.h>
#include <SPI.h>
#include <BH1750.h>
#include <Bounce2.h>
#include "Wgs.h"
//#define MY_BME_ENABLED
#ifdef MY_BME_ENABLED
#include <BME280I2C.h> // From Library Manager, comes with the BME280-lib by Tyler Glenn
//#define MY_FORECAST
#endif

#include <Wire.h> //warum andere Schreibweise? => Arduino IDE sagt: "Wire.h" bedeutet: liegt im gleichen Verzeichnis <Wire.h> liegt im lib-Verzeichnis. M.E. ist es als <Wire.h> richtig
#include <MySensors.h>

//für die millis()-Berechnung, wann wieder gesendet werden soll
unsigned long SEND_FREQUENCY = 180000; // Sleep time between reads (in milliseconds)
unsigned long lastSend = 0;

#define CHILD_ID_LIGHT 0
#define CHILD_ID_RAIN 1   // Id of the sensor child
#define COVER_0_ID 2
#define MAX_COVERS 2

#ifdef MY_BME_ENABLED
#define BARO_CHILD 10
#define TEMP_CHILD 11
#define HUM_CHILD 12
BME280I2C bme;
unsigned long lastSendBme = 0;
//#define SEALEVELPRESSURE_HPA (1013.25)
#endif

// Input Pins for covers Up/Down
// UP-Button, DOWN-Button
const uint8_t INPUT_PINS[][2] = { {4,3}, {6,7}};

//Notfall
const uint8_t SwEmergency = 5;

// ********************* EEPROM DEFINES **********************************************
#define EEPROM_OFFSET_SKETCH      512                   //  Address start where we store our data (before is mysensors stuff)
#define EEPROM_POSITION           EEPROM_OFFSET_SKETCH  //  Address of shutter last position
#define EEPROM_TRAVELUP           (EEPROM_POSITION+1)   //  Address of travel time for Up. 16bit value
#define EEPROM_TRAVELDOWN         (EEPROM_TRAVELUP+2)   //  Address of travel time for Down. 16bit value

// ********************* CALIBRATION DEFINES *****************************************
//#define START_POSITION            0                     // Shutter start position in % if no parameter in memory

uint16_t travelUpTime[MAX_COVERS] = {16000};   // in ms, time measured between 0-100% for Up. it's initialization value. if autocalibration used, these will be overriden
uint16_t travelDownTime[MAX_COVERS] = {16000}; // in ms, time measured between 0-100% for Down

bool UpStates[MAX_COVERS] = {0};
bool DownStates[MAX_COVERS] = {0};
bool ReverseStates[MAX_COVERS] = {0};
bool receivedLastLevel[MAX_COVERS] = {false};
bool EmergencyEnable[MAX_COVERS] = {false};

// Output Pins
// Cover_ON, Cover_DOWN,
const uint8_t OUTPUT_PINS[MAX_COVERS][2] = {{12,10}, {13,11}} ;
/*const int JalOn = 10;   // activates relais 2
const int JalDown = 12; // activates relais1+2
const int MarkOn = 11; // activates relais 4
const int MarkDown = 13; // activates relais 3+4
 */

Wgs Cover[MAX_COVERS];

uint8_t State[MAX_COVERS] = {0};
uint8_t oldState[MAX_COVERS] = {0};
uint8_t status[MAX_COVERS] = {0};
uint8_t oldStatus[MAX_COVERS] = {0};

MyMessage upMessage(COVER_0_ID, V_UP);
MyMessage downMessage(COVER_0_ID, V_DOWN);
MyMessage stopMessage(COVER_0_ID, V_STOP);
MyMessage statusMessage(COVER_0_ID, V_STATUS);
MyMessage msgRain(CHILD_ID_RAIN, V_RAIN);
MyMessage msgLux(CHILD_ID_LIGHT, V_LIGHT_LEVEL);

Bounce debounce[MAX_COVERS][2];
Bounce debounceMarkEmergency  = Bounce();

BH1750 lightSensor;
uint16_t lastlux = 0;

#ifdef MY_BME_ENABLED
float lastPressure = -1;
float lastTemp = -1;
float lastHum = -1;
float temperature(NAN), humidity(NAN), pressureBme(NAN);
uint8_t pressureUnit(1);                                          
// unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi

MyMessage tempMsg(TEMP_CHILD, V_TEMP);
MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);
MyMessage humMsg(HUM_CHILD, V_HUM);
#endif
#ifdef MY_FORECAST
//bme: Value according to MySensors for forecast accuracy; do not change
unsigned long bmeDelayTime = 60000;

const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
uint8_t lastForecast = -1;
const uint8_t LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];
// this CONVERSION_FACTOR is used to convert from Pa to kPa in forecast algorithm
// get kPa/h be dividing hPa by 10
#define CONVERSION_FACTOR (1.0/10.0)

int minuteCount = 0;
bool firstRound = true;
// average value is used in forecast algorithm.
float pressureAvg;
// average after 2 hours is used as reference value for the next iteration.
float pressureAvg2;

float dP_dt;
MyMessage forecastMsg(BARO_CHILD, V_FORECAST);
#endif

bool metric = true;

void driveToTarget(const uint8_t cover, const uint8_t targetPos) 
{
/* Ablauf:
- Position bestimmen => rauf bzw. runter?
- Soll-Laufzeit errechnen
- ggf. stoppen und Gegenrichtung veranlassen
- neue _finish_time errechnen und setzen
*/  
  
  //Position bestimmen
  //if 

}

void sendState(int val1, int sensorID) {
  // Send current state and status to gateway.
  send(upMessage.setSensor(sensorID).set(State[val1] == 2));
  send(downMessage.setSensor(sensorID).set(State[val1] == 4));
  send(stopMessage.setSensor(sensorID).set(State[val1] == 0));
  send(statusMessage.setSensor(sensorID).set(status[val1]));
}

void before()
{
  // Initialize In-/Outputs
  for (uint8_t i = 0; i < MAX_COVERS; i++) {
    travelUpTime[i] = readEeprom(EEPROM_POSITION+4*i);
    travelDownTime[i] = readEeprom(EEPROM_POSITION+4*i+2);
    Cover[i] = Wgs(OUTPUT_PINS[i][0],OUTPUT_PINS[i][1],travelUpTime[i]);
      for (uint8_t j=0; j<2; j++) {
        pinMode(OUTPUT_PINS[i][j], OUTPUT);
        digitalWrite(OUTPUT_PINS[i][j], HIGH);
        //pinMode(INPUT_PINS[i][j], INPUT_PULLUP);
        debounce[i][j] = Bounce();
        debounce[i][j].attach(INPUT_PINS[i][j], INPUT_PULLUP);
        debounce[i][j].interval(5);
      }
    EmergencyEnable[i] = loadState(COVER_0_ID+i);
  }
  //pinMode(SwEmergency, INPUT_PULLUP);
  debounceMarkEmergency.attach(SwEmergency, INPUT_PULLUP);
  debounceMarkEmergency.interval(5);

  Wire.begin();
  lightSensor.begin();
#ifdef MY_BME_ENABLED
  bme.begin();
#endif
}

void presentation() {
  sendSketchInfo(SN, SV);
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  present(CHILD_ID_RAIN, S_RAIN);
  for (uint8_t i = 0; i < MAX_COVERS; i++) {
    present(COVER_0_ID+i, S_COVER);
    present(COVER_0_ID+i, S_CUSTOM);
  }
#ifdef MY_BME_ENABLED
  present(BARO_CHILD, S_BARO);
  present(TEMP_CHILD, S_TEMP);
  present(HUM_CHILD, S_HUM);
#endif
}

void setup() {
  for (uint8_t i = 0; i < MAX_COVERS; i++) {
    //sendState(i, COVER_0_ID+i);
    request(COVER_0_ID+i, V_PERCENTAGE);
    request(COVER_0_ID+i, V_VAR2);
    request(COVER_0_ID+i, V_VAR3);
  }
  metric = getControllerConfig().isMetric;
}

void loop()
{
  bool button[MAX_COVERS][2];
  bool bounceUpdate[MAX_COVERS] = {false}; //true, if button pressed
  debounceMarkEmergency.update();
  bool emergency = debounceMarkEmergency.read() == LOW; //Current use: in case of rain

  for (uint8_t i = 0; i < MAX_COVERS; i++) {
    for (uint8_t j=0; j<2; j++) {
      debounce[i][j].update();
    if (!bounceUpdate[i]){
      bounceUpdate[i] = debounce[i][j].read() == HIGH;
    }
      button[i][j] = debounce[i][j].read() == LOW;
  }
  if (EmergencyEnable[i] == 1) Cover[i].setDisable(emergency);
  }

  unsigned long currentTime = millis();

  // Only send values at a maximum frequency
  if (currentTime - lastSend > SEND_FREQUENCY) {
    lastSend = currentTime;
    uint16_t lux = lightSensor.readLightLevel();// Get Lux value
    if (lux != lastlux) {
      send(msgLux.set(lux));
      lastlux = lux;
#ifdef MY_DEBUG_LOCAL
      Serial.print("lux:");
      Serial.println(lux);
#endif
    }
    send(msgRain.set(emergency));
#ifdef MY_BME_ENABLED
    bme.read(pressureBme, temperature, humidity, metric, pressureUnit); //Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
    if (isnan(temperature)) {
#ifdef MY_DEBUG_LOCAL
      Serial.println("Failed reading temperature");
#endif
    } else if (temperature != lastTemp) {
      // Only send temperature if it changed since the last measurement
      lastTemp = temperature;
      send(tempMsg.set(temperature, 1));
#ifdef MY_DEBUG_LOCAL
      Serial.print("T: ");
      Serial.println(temperature);
#endif
    }
    
    if (isnan(humidity)) {
#ifdef MY_DEBUG_LOCAL
      Serial.println("Failed reading humidity");
#endif
    } else if (humidity != lastHum) {
    // Only send humidity if it changed since the last measurement
      lastHum = humidity;
      send(humMsg.set(humidity, 1));
#ifdef MY_DEBUG
      Serial.print("H: ");
      Serial.println(humidity);
#endif
    }

#ifndef MY_FORECAST
    if (isnan(pressureBme)) {
#ifdef MY_DEBUG_LOCAL
      Serial.println("Failed reading pressure");
#endif
    if (pressureBme != lastPressure) {
      send(pressureMsg.set(pressureBme, 2));
      lastPressure = pressureBme;
    }
  }
#endif

#ifdef MY_FORECAST
  if (currentTime - lastSendBme > bmeDelayTime) {
    int forecast = sample(pressureBme);
    if (pressureBme != lastPressure) {
      send(pressureMsg.set(pressureBme, 2));
      lastPressure = pressureBme;
    }
    if (forecast != lastForecast){
      send(forecastMsg.set(weather[forecast]));
      lastForecast = forecast;
    }
  }
#endif
#endif
  }
  //State[0]=Cover[0].loop(button_mark_up, button_mark_down);
  for (uint8_t i = 0; i < MAX_COVERS; i++) {
    //State[i]=Cover[i].loop(button[i][0],button[i][1] );
    Cover[i].loop(button[i][0],button[i][1] );
    State[i]=Cover[i].getState();
    if ( State[i] != oldState[i]||status[i] != oldStatus[i]) {
      sendState(i, COVER_0_ID+i);
/*
 * Hier könnte man einen Timer einfügen, der die Zeit erfaßt,
 * die das jeweilige Cover fährt und daraus einen %-Wert errechnen.
 * Den könnte man dann bei Überschreitung einer gewissen Hysterese
 * senden bzw. dann, wenn ein Stop-Befehl kommt.
 * Weiter könnte man darüber vergleichen, ob das jeweilige Cover
 * seinen Sollwert erreicht hat und dann die Fahrt abbrechen.
 * Dazu bräuchte man aber (mindestens) die Fahrtdauern hoch bzw. runter,
 * die man in VAR1 und VAR2 vom Controller erfragen könnte bzw.
 * mit einem Standardwert vorbelegen.
 * Zielwert bei Tastendruck löschen?
 */
      oldState[i] = State[i];
      oldStatus[i] = status[i];
#ifdef MY_DEBUG_LOCAL
      Serial.print("Button press C ");
      Serial.println(i+COVER_0_ID);
      Serial.print("Return: ");
      Serial.println(State[i]);
#endif
    }
  }
}

void receive(const MyMessage &message) {
  if (message.sensor >= COVER_0_ID && message.sensor <= COVER_0_ID+MAX_COVERS) {
    uint8_t cover_i = message.sensor-COVER_0_ID;
    if (message.isAck()) {
#ifdef MY_DEBUG_LOCAL
    Serial.println(F("Ack child1 from gw rec."));
#endif
    }
    if (message.type == V_DIMMER) { // This could be M_ACK_VARIABLE or M_SET_VARIABLE
      int val = message.getInt();
      if (!receivedLastLevel[cover_i]) {
        receivedLastLevel[cover_i] = true;  
      }
      else {/*
        * Die State-Bezüge sind "geraten", es sollte lt cpp sein:
        * const int STATE_UNKNOWN = 0;
          const int STATE_ENABLED = 1;
          const int STATE_DISABLING = 2;
          const int STATE_DISABLED = 3;
          const int STATE_ENABLING = 4;
        */
        if (val < 0) {
        //Stop
          Cover[cover_i].stopMovement(0);
#ifdef MY_DEBUG_ACTUAL
          Serial.print("GW Message stop: ");
          Serial.println(val);
#endif
        }
    
        else if (val == 0 && State[cover_i] != 2 && State[cover_i] != 3) {
        //Down
          if (Cover[cover_i].getState() != 0) {
            State[cover_i] = Cover[cover_i].loop(true, false);
          }
          State[cover_i] = Cover[cover_i].loop(true, false);
#ifdef MY_DEBUG_LOCAL
          Serial.print("GW Message down: ");
          Serial.println(val);
#endif
        }
    
        else if (val == 100 && State[cover_i] != 1 && State[cover_i] != 4) {
        //Up
#ifdef MY_DEBUG_ACTUAL
          Serial.println(message.getInt());
          Serial.print("Cover state: ");
          Serial.println(Cover[cover_i].getState());
#endif
          if (Cover[cover_i].getState() != 0) {
            State[cover_i] = Cover[cover_i].loop(false, true);
          }
          State[cover_i] = Cover[cover_i].loop(false, true);
#ifdef MY_DEBUG_LOCAL
          Serial.print("GW Msg up: ");
          Serial.println(val);
#endif
        } 
        else {
          driveToTarget(cover_i,val);
        }
      } 
      
    }

    else if (message.type == V_UP && State[cover_i] != 1 && State[cover_i] != 4) {
      if (Cover[cover_i].getState() != 0) {
        State[cover_i] = Cover[cover_i].loop(false, true);
      }
      State[cover_i] = Cover[cover_i].loop(false, true);
      //sendState();
#ifdef MY_DEBUG_LOCAL
      Serial.print("GW Msg up, C ");
      Serial.println(message.sensor);
#endif
    }
    else if (message.type == V_DOWN && State[cover_i] != 2 && State[cover_i] != 3) {
      if (Cover[cover_i].getState() != 0) {
        State[cover_i] = Cover[cover_i].loop(true, false);
      }
      State[cover_i] = Cover[cover_i].loop(true, false);
#ifdef MY_DEBUG_LOCAL
      Serial.print(F("GW Msg down, C "));
      Serial.println(message.sensor);
#endif
    }
    else if (message.type == V_STOP) {
      Cover[cover_i].stopMovement(0);
#ifdef MY_DEBUG_LOCAL
      Serial.print(F("GW Msg stop, C "));
      Serial.println(message.sensor);
#endif
    }
    else if (message.type == V_VAR1){
      EmergencyEnable[cover_i] = message.getBool();    
      if (EmergencyEnable[cover_i] != loadState(cover_i)) {
       saveState(message.sensor,EmergencyEnable[cover_i]);
      }
#ifdef MY_DEBUG_ACTUAL
      Serial.print("Cover emergency state: ");
      Serial.println(EmergencyEnable[cover_i]);
#endif
    }
    else if (message.type == V_VAR2 && message.getInt() > 0) {
      travelUpTime[cover_i] = message.getInt();    
      if (travelUpTime[cover_i] != readEeprom(EEPROM_POSITION+4*cover_i)) {
       writeEeprom(EEPROM_POSITION+4*cover_i+2, travelUpTime[cover_i]);
      }
    }
    else if (message.type == V_VAR3 && message.getInt() > 0) {
      travelDownTime[cover_i] = message.getInt();    
      if (travelDownTime[cover_i] != readEeprom(EEPROM_POSITION+4*cover_i+2)) {
       writeEeprom(EEPROM_POSITION+4*cover_i+2, travelDownTime[cover_i]); 
      }
    }
  }
}
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

#ifdef MY_FORECAST
enum FORECAST
{
    STABLE = 0,            // "Stable Weather Pattern"
    SUNNY = 1,            // "Slowly rising Good Weather", "Clear/Sunny "
    CLOUDY = 2,            // "Slowly falling L-Pressure ", "Cloudy/Rain "
    UNSTABLE = 3,        // "Quickly rising H-Press",     "Not Stable"
    THUNDERSTORM = 4,    // "Quickly falling L-Press",    "Thunderstorm"
    UNKNOWN = 5            // "Unknown (More Time needed)
};

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++) {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;
  return lastPressureSamplesAverage;
}


// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185)
  {
    minuteCount = 6;
  }

  if (minuteCount == 5)
  {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else
    {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) //first time initial 3 hour
    {
      dP_dt = change; //note this is for t = 1 hour
    }
    else
    {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else
    {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else
    {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    }
    else
    {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 3; // note this is for t = 3 hour
    }
    else
    {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
  {
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25))
  {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25)
  {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
  {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt >(-0.05)) && (dP_dt < 0.05))
  {
    forecast = STABLE;
  }
  else
  {
    forecast = UNKNOWN;
  }

  // uncomment when debugging
  //Serial.print(F("Forecast at minute "));
  //Serial.print(minuteCount);
  //Serial.print(F(" dP/dt = "));
  //Serial.print(dP_dt);
  //Serial.print(F("kPa/h --> "));
  //Serial.println(weather[forecast]);

  return forecast;
}
#endif

/* ======================================================================
Function: writeEeprom
Purpose : Write uint16_t values in eeprom for (max 65535)
Input   : - pos : start position in eeprom
          - value : 16bits value to write 
Comments: 
====================================================================== */
void writeEeprom(uint16_t pos, uint16_t value) {
  
    // function for saving the values to the internal EEPROM
    // value = the value to be stored (as int)
    // pos = the first byte position to store the value in
    // only two bytes can be stored with this function (max 32.767)
    EEPROM.write(pos, ((uint16_t)value >> 8 ));
    pos++;
    EEPROM.write(pos, (value & 0xff));
    
}
/* ======================================================================
Function: readEeprom
Purpose : read uint16_t values in eeprom for (max 65535)
Input   :  - pos : start position in eeprom
Output  :  - return 16bits value
Comments: 
====================================================================== */
uint16_t readEeprom(uint16_t pos) {
  
    // function for reading the values from the internal EEPROM
    // pos = the first byte position to read the value from 
    uint16_t hiByte;
    uint16_t loByte;

    hiByte = EEPROM.read(pos) << 8;
    pos++;
    loByte = EEPROM.read(pos);
    return (hiByte | loByte);
    
}
