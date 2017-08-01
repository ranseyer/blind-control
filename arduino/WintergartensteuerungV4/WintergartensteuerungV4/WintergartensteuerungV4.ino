/*
 * Changelog: Kommentare zum weiteren Vorgehen eingefügt
 * BME280: neuer Anlauf
 * Weitestgehend auf Arrays umgebaut
 */

#define SN "DoubleCover"
#define SV "0.4.0"

//#define MY_DEBUG
//#define MY_DEBUG_LOCAL //Für lokale Debug-Ausgaben
// Enable RS485 transport layer
#define MY_RS485
//#define MY_RS485_HWSERIAL Serial
// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 2
// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600
#define MY_NODE_ID 111
#define MY_TRANSPORT_WAIT_READY_MS 2000
#include <Arduino.h>
#include <SPI.h>
#include <BH1750.h>
#include <Bounce2.h>
#include "Wgs.h"
#include <Wire.h> //Arduino IDE sagt: "Wire.h" bedeutet: liegt im gleichen Verzeichnis <Wire.h> liegt im lib-Verzeichnis. M.E. ist es als <Wire.h> richtig
#include <Time.h>
#include <BME280I2C.h> // From Library Manager, comes with the BME280-lib by Tyler Glenn
//#define MY_FORECAST
#include <MySensors.h>

//für die millis()-Berechnung, wann wieder gesendet werden soll
unsigned long SEND_FREQUENCY = 180000; // Sleep time between reads (in milliseconds)
unsigned long lastSend = 0;

#define CHILD_ID_LIGHT 0
#define CHILD_ID_RAIN 1   // Id of the sensor child
#define COVER_0_ID 2
#define MAX_COVERS 2

#define BARO_CHILD 10
#define TEMP_CHILD 11
#define HUM_CHILD 12

BME280I2C bme;
unsigned long lastSendBme = 0;
//#define SEALEVELPRESSURE_HPA (1013.25)





// Input Pins for covers Up/Down
// UP-Button, DOWN-Button
const uint8_t INPUT_PINS[][2] = { {6,7}, {4,3}};
const int SwEmergency = 5;
/*const int SwMarkUp = 6;
const int SwMarkDown = 7;
// Input Pins for Switch Jalosie Up/Down
const int SwJalUp = 4;
const int SwJalDown = 3;
*/


// Output Pins
// Cover_ON, Cover_DOWN,

//const unsigned long ON_Time_Max = 16000;
const uint8_t OUT_INFOS[][2] = {{12,10}, {13,11}} ;
/* Output Pins
//dto zum obigen Array
const int JalOn = 12;   // activates relais 2
const int JalDown = 10; // activates relais1+2
//const int JalRevers = 12;
const int MarkOn = 13; // activates relais 4
const int MarkDown = 11; // activates relais 3+4*/



bool UpStates[MAX_COVERS] = {0};
bool DownStates[MAX_COVERS] = {0};
bool ReverseStates[MAX_COVERS] = {0};
bool EmergencyState = 0;

Wgs Cover[MAX_COVERS];
/*for (int i = 0; i < MAX_COVERS; i++) {
  Cover[i] = OUT_INFOS[i];
}*/
BH1750 lightSensor;
uint16_t lastlux = 0;

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

/*Auck keine Ahnung, ob Bounce mit einem Anlegen
 * über ein Array klarkäme
 */
Bounce debounce[MAX_COVERS][1];
/*Bounce debounceJalDown  = Bounce();
Bounce debounceMarkUp    = Bounce();
Bounce debounceMarkDown  = Bounce();*/

Bounce debounceMarkEmergency  = Bounce();

float lastPressure = -1;
float lastTemp = -1;
float lastHum = -1;

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
float temperature(NAN), humidity(NAN), pressureBme(NAN);
uint8_t pressureUnit(1);
// unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi

MyMessage tempMsg(TEMP_CHILD, V_TEMP);
MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);
MyMessage humMsg(HUM_CHILD, V_HUM);

void before()
{
  // Initialize In-/Outputs
  for (uint8_t i = 0; i < MAX_COVERS; i++) {
  Cover[i] = Wgs(OUT_INFOS[i][0],OUT_INFOS[i][1],16000);
    for (uint8_t j=0; j<2; j++) {
      pinMode(OUT_INFOS[i][j], OUTPUT);
      digitalWrite(OUT_INFOS[i][j], HIGH);
      pinMode(INPUT_PINS[i][j], INPUT_PULLUP);
      debounce[i][j] = Bounce();
      debounce[i][j].attach(INPUT_PINS[i][j]);
      debounce[i][j].interval(5);
    }
  }
  pinMode(SwEmergency, INPUT_PULLUP);
  debounceMarkEmergency.attach(SwEmergency);
  debounceMarkEmergency.interval(5);

  Wire.begin();
  lightSensor.begin();
  bme.begin();
}

void presentation() {
  sendSketchInfo(SN, SV);
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  present(CHILD_ID_RAIN, S_RAIN);
  for (uint8_t i = 0; i < MAX_COVERS; i++) {
    present(COVER_0_ID+i, S_COVER);
    present(COVER_0_ID+i, S_CUSTOM);
  }
  present(BARO_CHILD, S_BARO);
  present(TEMP_CHILD, S_TEMP);
  present(HUM_CHILD, S_HUM);
}

void setup() {
  for (uint8_t i = 0; i < MAX_COVERS; i++) {
    sendState(i, COVER_0_ID+i);
  }
  metric = getControllerConfig().isMetric;
}

void loop()
{
  bool button[MAX_COVERS-1][1];
  for (uint8_t i = 0; i < MAX_COVERS; i++) {
    for (uint8_t j=0; j<2; j++) {
      button[i][j] = digitalRead(INPUT_PINS[i][j]) == LOW;
    }
  }

  bool emergency = digitalRead(SwEmergency) == LOW; //Current use: in case of rain

  Cover[0].setDisable(emergency);

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
#endif
  }

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
#endif
  }

  //State[0]=Cover[0].loop(button_mark_up, button_mark_down);
  for (uint8_t i = 0; i < MAX_COVERS; i++) {
    State[i]=Cover[i].loop(button[i][0],button[i][1] );
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
    if (message.isAck()) {
#ifdef MY_DEBUG_LOCAL
    Serial.println(F("Ack child1 from gw rec."));
#endif
    }
    if (message.type == V_DIMMER) { // This could be M_ACK_VARIABLE or M_SET_VARIABLE
      int val = message.getInt();
      /*
       * Die State-Bezüge sind "geraten", es sollte lt cpp sein:
       * const int STATE_UNKNOWN = 0;
          const int STATE_ENABLED = 1;
          const int STATE_DISABLING = 2;
          const int STATE_DISABLED = 3;
          const int STATE_ENABLING = 4;
       */
      if (val < 50 && State[message.sensor-COVER_0_ID] != 2 && State[message.sensor-COVER_0_ID] != 3) {
      //Down
        if (State[message.sensor-COVER_0_ID] != 0) {
          Cover[message.sensor-COVER_0_ID].loop(true, false);
        }
        Cover[message.sensor-COVER_0_ID].loop(true, false);
#ifdef MY_DEBUG_LOCAL
        Serial.print("GW Message up: ");
        Serial.println(val);
#endif
      }
      else if (val == 50) {
      //Stop
        Cover[message.sensor-COVER_0_ID].loop(false, false);
#ifdef MY_DEBUG_LOCAL
        Serial.print("GW Message stop: ");
        Serial.println(val);
#endif
      }
      else if (val >50 && State[message.sensor-COVER_0_ID] != 1 && State[message.sensor-COVER_0_ID] != 4) {
      //Up
        if (State[message.sensor-COVER_0_ID] != 0) {
          Cover[message.sensor-COVER_0_ID].loop(false, true);
        }
        Cover[message.sensor-COVER_0_ID].loop(false, true);
#ifdef MY_DEBUG_LOCAL
        Serial.print("GW Msg down: ");
        Serial.println(val);
#endif
      }
    }

    if (message.type == V_UP && State[message.sensor-COVER_0_ID] != 1 && State[message.sensor-COVER_0_ID] != 4) {
      Cover[message.sensor-COVER_0_ID].loop(true, false);
      //sendState();
#ifdef MY_DEBUG_LOCAL
      Serial.print("GW Msg up, C ");
      Serial.println(message.sensor);
#endif
    }
    if (message.type == V_DOWN && State[message.sensor-COVER_0_ID] != 2 && State[message.sensor-COVER_0_ID] != 3) {
      Cover[message.sensor-COVER_0_ID].loop(false, true);
#ifdef MY_DEBUG_LOCAL
      Serial.print(F("GW Msg down, C "));
      Serial.println(message.sensor);
#endif
    }
    if (message.type == V_STOP) {
      Cover[message.sensor-COVER_0_ID].loop(false, false);
#ifdef MY_DEBUG_LOCAL
      Serial.print(F("GW Msg stop, C "));
      Serial.println(message.sensor);
#endif
    }
  }
}

enum State {
  IDLE,
  UP, // Window covering. Up.
  DOWN, // Window covering. Down.
};

void sendState(int val1, int sensorID) {
  // Send current state and status to gateway.
  send(upMessage.setSensor(sensorID).set(State[val1] == UP));
  send(downMessage.setSensor(sensorID).set(State[val1] == DOWN));
  send(stopMessage.setSensor(sensorID).set(State[val1] == IDLE));
  send(statusMessage.setSensor(sensorID).set(status[val1]));
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
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
  {
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
