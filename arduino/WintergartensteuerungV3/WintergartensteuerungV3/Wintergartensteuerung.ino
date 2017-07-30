/*
 * Changelog: Kommentare zum weiteren Vorgehen eingefügt
 * BME280 eingefügt mit pressure und forecast
 */

#define SN "DoubleCover"
#define SV "0.3.1"

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
// For RTC
#include "Wire.h" //warum andere Schreibweise ?
#include <BME280I2C.h> // From Library Manager, comes with the BME280-lib by Tyler Glenn

#include <MySensors.h>

//für die millis()-Berechnung, wann wieder gesendet werden soll
unsigned long SEND_FREQUENCY = 180000; // Sleep time between reads (in milliseconds)
unsigned long lastSend = 0;

#define CHILD_ID_LIGHT 0
#define CHILD_ID_RAIN 1   // Id of the sensor child
#define First_CHILD_ID_COVER 2
#define MAX_COVERS 2

#define BARO_CHILD 10
#define TEMP_CHILD 11
#define HUM_CHILD 12

BME280I2C bme;
const float ALTITUDE = 200; // <-- adapt this value to your location's altitude (in m). Use your smartphone GPS to get an accurate value!
unsigned long lastSendBme = 0;
//#define SEALEVELPRESSURE_HPA (1013.25)

// Input Pins for Switch Markise Up/Down
/*Könnte man für jeweils die zu einem Cover gehörenden PINs auch über eine Array-Funktion lösen,
 damit man dann zur Initialisierung, Abfrage etc. einfach eine Schleife drüberlegen kann.
 Also etwa so:
 const int INPUT_PINS[MAX_COVERS] = {
   {3,4},
   {7,6}
 };
*/
const int SwMarkUp = 3;
const int SwMarkDown = 4;
// Input Pins for Switch Jalosie Up/Down
const int SwJalUp = 7;
const int SwJalDown = 6;
//Notfall
const int SwEmergency = 5;

// Output Pins
//dto zum obigen Array
const int JalOn = 10;   // activates relais 2
const int JalDown = 12; // activates relais1+2
//const int JalRevers = 12;
const int MarkOn = 11; // activates relais 4
const int MarkDown = 13; // activates relais 3+4

/*Die States könnten vermutlich auch bool sein (Speicher...)
und die Cover-spezifischen könnte man in einem Array organisieren
 */
int MarkUpState = 0;
int MarkDownState = 0;
int JalUpState = 0;
int JalDownState = 0;
int JalReverseState = 0;
int EmergencyState = 0;

//autostart
const int autostart_time = 9;

const int autostart_check_delay = 200; //in ticks
int autostart_check_tick = 200; //in ticks
boolean autostart_done = false;

/*Allerdings habe ich keine Idee, wie man die Devices
Innerhalb einer Schleife sinnvoll anlegen kann.
Vielleicht hat Dein Sohn da eine Idee*/
//Wgs mark(MarkOn, MarkDown, 55000);
//Wgs jal(JalOn, JalDown, 55000);
// Kürzer für Test
Wgs mark(MarkOn, MarkDown, 6000);
Wgs jal(JalOn, JalDown, 6000);


BH1750 lightSensor;
uint16_t lastlux = 0;

byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

enum State {
  IDLE,
  UP, // Window covering. Up.
  DOWN, // Window covering. Down.
};
int State[MAX_COVERS] = {0};
int oldState[MAX_COVERS] = {0};
int status[MAX_COVERS] = {0};
int oldStatus[MAX_COVERS] = {0};

//eine MyMessage-Funktion sollte ausreichen; Rest geht (hoffentlich) über Indexierung
MyMessage upMessage(First_CHILD_ID_COVER, V_UP);
MyMessage downMessage(First_CHILD_ID_COVER, V_DOWN);
MyMessage stopMessage(First_CHILD_ID_COVER, V_STOP);
MyMessage statusMessage(First_CHILD_ID_COVER, V_STATUS);
MyMessage msgRain(CHILD_ID_RAIN, V_RAIN);
MyMessage msgLux(CHILD_ID_LIGHT, V_LIGHT_LEVEL);

/*Auck keine Ahnung, ob Bounce mit einem Anlegen
 * über ein Array klarkäme
 */
Bounce debounceJalUp    = Bounce();
Bounce debounceJalDown  = Bounce();
Bounce debounceMarkEmergency  = Bounce();
Bounce debounceMarkUp    = Bounce();
Bounce debounceMarkDown  = Bounce();

//bme: Value according to MySensors for forecast accuracy
unsigned long bmeDelayTime = 60000;

const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
    STABLE = 0,            // "Stable Weather Pattern"
    SUNNY = 1,            // "Slowly rising Good Weather", "Clear/Sunny "
    CLOUDY = 2,            // "Slowly falling L-Pressure ", "Cloudy/Rain "
    UNSTABLE = 3,        // "Quickly rising H-Press",     "Not Stable"
    THUNDERSTORM = 4,    // "Quickly falling L-Press",    "Thunderstorm"
    UNKNOWN = 5            // "Unknown (More Time needed)
};

float lastPressure = -1;
float lastTemp = -1;
float lastHum = -1;
int lastForecast = -1;

const int LAST_SAMPLES_COUNT = 5;
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
bool metric = true;
MyMessage tempMsg(TEMP_CHILD, V_TEMP);
MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);
MyMessage forecastMsg(BARO_CHILD, V_FORECAST);
MyMessage humMsg(HUM_CHILD, V_HUM);

void sendState(int val1, int sensorID) {
  // Send current state and status to gateway.
  send(upMessage.setSensor(sensorID).set(State[val1] == UP));
  send(downMessage.setSensor(sensorID).set(State[val1] == DOWN));
  send(stopMessage.setSensor(sensorID).set(State[val1] == IDLE));
  send(statusMessage.setSensor(sensorID).set(status[val1]));
}

void before()
{
  /*
   * Mal exemplarisch für Markise umsortiert, wie man das im Ramhen eines Arrays aufrufen würde
   */

  // Initialize In-/Outputs
  pinMode(MarkOn, OUTPUT);
  pinMode(MarkDown, OUTPUT);
  digitalWrite(MarkOn, HIGH);
  digitalWrite(MarkDown, HIGH);
  pinMode(JalOn, OUTPUT);
  pinMode(JalDown, OUTPUT);
  digitalWrite(JalOn, HIGH);
  digitalWrite(JalDown, HIGH);

  pinMode(SwJalUp, INPUT_PULLUP);
  pinMode(SwJalDown, INPUT_PULLUP);
  pinMode(SwMarkUp, INPUT_PULLUP);
  pinMode(SwMarkDown, INPUT_PULLUP);
  pinMode(SwEmergency, INPUT_PULLUP);

  // After setting up the button, setup debouncer
  debounceMarkUp.attach(SwMarkUp);
  debounceMarkUp.interval(5);
  debounceMarkDown.attach(SwMarkDown);
  debounceMarkDown.interval(5);
  debounceJalUp.attach(SwJalUp);
  debounceJalUp.interval(5);
  debounceJalDown.attach(SwJalDown);
  debounceJalDown.interval(5);
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
  for (int i = 0; i < MAX_COVERS; i++) {
    present(First_CHILD_ID_COVER+i, S_COVER);
    present(First_CHILD_ID_COVER+i, S_CUSTOM);
  }
  present(BARO_CHILD, S_BARO);
  present(TEMP_CHILD, S_TEMP);
  present(HUM_CHILD, S_HUM);
}

void setup() {
  for (int i = 0; i < MAX_COVERS; i++) {
    sendState(i, First_CHILD_ID_COVER+i);
  }
  metric = getControllerConfig().isMetric;
}

void loop()
{
  //Serial.print("Loop/");

  bool button_mark_up = digitalRead(SwMarkUp) == LOW;
  bool button_mark_down = digitalRead(SwMarkDown) == LOW;
  bool button_jal_up = digitalRead(SwJalUp) == LOW;
  bool button_jal_down = digitalRead(SwJalDown) == LOW;
  bool emergency = digitalRead(SwEmergency) == LOW; //Current use: in case of rain

  mark.setDisable(emergency);

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
    
    float temperature = bme.temp(metric);
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

    float humidity = bme.hum();
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
  }

  if (currentTime - lastSendBme > bmeDelayTime) {
    float pressure_local = bme.pres();                    // Get pressure at current location
    float pressure = pressure_local/pow((1.0 - ( ALTITUDE / 44330.0 )), 5.255); // Adjust to sea level pressure using user altitude
    int forecast = sample(pressure);
    if (pressure != lastPressure) {
      send(pressureMsg.set(pressure, 2));
      lastPressure = pressure;
    }

    if (forecast != lastForecast){
      send(forecastMsg.set(weather[forecast]));
      lastForecast = forecast;
    }
  }



/*  
 * Braucht es den Autostart-Teil bei zentraler Steuerung?   
 //Autostart code
  autostart_check_tick++;
  if(autostart_check_tick >= autostart_check_delay){
    autostart_check_tick = 0;

    // Darf das mehrfach sein ?
    byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

    if(autostart_done){ //Already done
      if(hour > autostart_time){
        autostart_done = false;
      }
    }else{
      if(hour == autostart_time &! emergency){
        button_mark_down = true;
        autostart_done = true;
      }
    }
  }
*/
  State[0]=mark.loop(button_mark_up, button_mark_down);
  State[1]=jal.loop(button_jal_up, button_jal_down);

  for (int i = 0; i < MAX_COVERS; i++) {
    if ( State[i] != oldState[i]||status[i] != oldStatus[i]) {
      sendState(i, First_CHILD_ID_COVER+i);
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
      Serial.println(i+First_CHILD_ID_COVER);
      Serial.print("Return: ");
      Serial.println(State[i]);
#endif
    }
  }
}

void receive(const MyMessage &message) {

  //Diesen Teil später doppeln für den 2. Cover, solange Indexierung nicht läuft
  if (message.sensor == First_CHILD_ID_COVER) {
      if (message.isAck()) {
      Serial.println("Ack child1 from gw rec.");
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
      if (val < 50 && State[message.sensor-First_CHILD_ID_COVER] != 2 && State[message.sensor-First_CHILD_ID_COVER] != 3) {
        //Up
        if (State[message.sensor-First_CHILD_ID_COVER] != 0) {
          mark.loop(true, false);
        }
        //bool button_mark_up=true;
        //bool button_mark_down=false;
        mark.loop(true, false);

#ifdef MY_DEBUG_LOCAL
    Serial.print("GW Message up: ");
    Serial.println(val);
#endif
      }
      else if (val == 50) {
        //Stop
        bool button_mark_up=false;
        bool button_mark_down=false;
        mark.loop(button_mark_up, button_mark_down);

#ifdef MY_DEBUG_LOCAL
    Serial.print("GW Message stop: ");
    Serial.println(val);
#endif
      }
      else if (val >50 && State[message.sensor-First_CHILD_ID_COVER] != 1 && State[message.sensor-First_CHILD_ID_COVER] != 4) {
       //Up
       if (State[message.sensor-First_CHILD_ID_COVER] != 0) {
          mark.loop(false, true);
        }
        //bool button_mark_up=false;
        //bool button_mark_down=true;
        mark.loop(false, true);

#ifdef MY_DEBUG_LOCAL
    Serial.print("GW Msg down: ");
    Serial.println(val);
#endif
      }
    }

    if (message.type == V_UP && State[message.sensor-First_CHILD_ID_COVER] != 1 && State[message.sensor-First_CHILD_ID_COVER] != 4) {
      // Set state to covering up and send it back to the gateway.
      State[message.sensor-First_CHILD_ID_COVER] = UP;
      bool button_mark_up=true;
      bool button_mark_down=false;
      mark.loop(button_mark_up, button_mark_down);
      //sendState();
#ifdef MY_DEBUG_LOCAL
    Serial.print("GW Msg up, C ");
    Serial.println(message.sensor);
#endif

    }
    if (message.type == V_DOWN && State[message.sensor-First_CHILD_ID_COVER] != 2 && State[message.sensor-First_CHILD_ID_COVER] != 3) {
      // Set state to covering up and send it back to the gateway.
      State[message.sensor-First_CHILD_ID_COVER] = DOWN;
      bool button_mark_up=false;
      bool button_mark_down=true;
      mark.loop(button_mark_up, button_mark_down);

#ifdef MY_DEBUG_LOCAL
    Serial.print("GW Msg down, C ");
    Serial.println(message.sensor);
#endif
    }

    if (message.type == V_STOP) {
      // Set state to idle and send it back to the gateway.
      State[message.sensor-First_CHILD_ID_COVER] = IDLE;
      bool button_mark_up=false;
      bool button_mark_down=false;
      mark.loop(button_mark_up, button_mark_down);
#ifdef MY_DEBUG_LOCAL
      Serial.print("GW Msg stop, C ");
      Serial.println(message.sensor);
#endif
    }
  //hier gehört die Doppelung für das 2. Cover hin
  }
}

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
