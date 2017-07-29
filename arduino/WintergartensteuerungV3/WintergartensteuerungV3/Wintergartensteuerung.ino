#define SN "DoubleCover"
#define SV "0.1.0"

#define MY_DEBUG
#define MY_DEBUG_LOCAL //Für lokale Debug-Ausgaben
// Enable RS485 transport layer
#define MY_RS485
// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 2
// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600
#define MY_NODE_ID 118
#define MY_TRANSPORT_WAIT_READY_MS 2000
#include <Arduino.h>
#include <SPI.h>
#include <BH1750.h>
#include <Bounce2.h>
#include "Wgs.h" //JR: Compilerfehler nach git clone
//#include <Bounce2.h>
// For RTC
#include "Wire.h" //warum andere Schreibweise ?
#define DS3231_I2C_ADDRESS 0x68
#include <MySensors.h>
//für die millis()-Berechnung, wann wieder gesendet werden soll
unsigned long SEND_FREQUENCY = 180000; // Sleep time between reads (in milliseconds)
unsigned long lastSend = 0;
unsigned long DISPLAY_UPDATE_FREQUENCY = 300; // (in milliseconds)
unsigned long lastUpdateDisplay = 0;

/*
 * Deaktiviert, da über loop() gelöst
// Initialize motion message
#define DIGITAL_INPUT_SENSOR 3   // The digital input you attached your rain sensor.  (Only 2 and 3 generates interrupt!)
#define SENSOR_INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
*/

// Input Pins for Switch Markise Up/Down
const int SwMarkUp = 3;
const int SwMarkDown = 4;
// Input Pins for Switch Jalosie Up/Down
const int SwJalUp = 7;
const int SwJalDown = 6;
//Notfall
const int SwEmergency = 5;

// Output Pins
const int JalOn = 10;
const int JalDown = 11;
//const int JalRevers = 12;
const int MarkOn = 12;
const int MarkDown = 13;

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

#define CHILD_ID_LIGHT 0
#define CHILD_ID_RAIN 1   // Id of the sensor child
#define First_CHILD_ID_COVER 2
#define MAX_COVERS 2
//#define CHILDCOVER1_ID_CONFIG_COVER 2 //ChildID darf vermutlich auch identisch sein, es wird bei cover ja kein VARx genutzt
//#define CHILDCOVER2_ID_CONFIG_COVER 3 //dann ist alles "beieinander"

Wgs mark(MarkOn, MarkDown, 55000);
Wgs jal(JalOn, JalDown, 55000);

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

// Braucht es nur einmal...
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
MyMessage upMessage(First_CHILD_ID_COVER, V_UP);  /// V_UP ???
MyMessage downMessage(First_CHILD_ID_COVER, V_DOWN);
MyMessage stopMessage(First_CHILD_ID_COVER, V_STOP);
MyMessage statusMessage(First_CHILD_ID_COVER, V_STATUS);
MyMessage msgRain(CHILD_ID_RAIN, V_RAIN);
MyMessage msgLux(CHILD_ID_LIGHT, V_LIGHT_LEVEL);

Bounce debounceJalUp    = Bounce();
Bounce debounceJalDown  = Bounce();
Bounce debounceMarkEmergency  = Bounce();
Bounce debounceMarkUp    = Bounce();
Bounce debounceMarkDown  = Bounce();


void sendState(int val1, int sensorID) {
  // Send current state and status to gateway.
  send(upMessage.setSensor(sensorID).set(State[val1] == UP));
  send(downMessage.setSensor(sensorID).set(State[val1] == DOWN));
  send(stopMessage.setSensor(sensorID).set(State[val1] == IDLE));
  send(statusMessage.setSensor(sensorID).set(status[val1]));
}

void before()
{
  // Initialize In-/Outputs
  pinMode(SwMarkUp, INPUT_PULLUP);

  pinMode(SwMarkDown, INPUT_PULLUP);
  pinMode(SwJalUp, INPUT_PULLUP);
  pinMode(SwJalDown, INPUT_PULLUP);
  pinMode(SwEmergency, INPUT_PULLUP);
  pinMode(MarkOn, OUTPUT);
  pinMode(MarkDown, OUTPUT);
  pinMode(JalOn, OUTPUT);
  pinMode(JalDown, OUTPUT);
  digitalWrite(MarkOn, HIGH);
  digitalWrite(MarkDown, HIGH);
  digitalWrite(JalOn, HIGH);
  digitalWrite(JalDown, HIGH);

  // After setting up the button, setup debouncer
  debounceJalUp.attach(SwJalUp);
  debounceJalUp.interval(5);
  debounceJalDown.attach(SwJalDown);
  debounceJalDown.interval(5);
  debounceMarkUp.attach(SwMarkUp);
  debounceMarkUp.interval(5);
  debounceMarkDown.attach(SwMarkDown);
  debounceMarkDown.interval(5);

  debounceMarkEmergency.attach(SwEmergency);
  debounceMarkEmergency.interval(5);/* initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion)
  //pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);
  //digitalWrite(DIGITAL_INPUT_SENSOR, HIGH);
  ISR-Funktionalität deaktiviert, wird durch die loop() geprüft und verarbeitet
  */
  //pulseCount = oldPulseCount = 0;
  //attachInterrupt(SENSOR_INTERRUPT, onPulse, CHANGE); //Unterstellt, es soll nur ein ja/nein-Signal sein

  Wire.begin();
  lightSensor.begin();
  //Serial.begin(57600);
}

void presentation() {
  sendSketchInfo(SN, SV);
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  present(CHILD_ID_RAIN, S_RAIN);
  for (int i = 0; i < MAX_COVERS; i++) {
    present(First_CHILD_ID_COVER+i, S_COVER);
    present(First_CHILD_ID_COVER+i, S_CUSTOM);
  }

}

void setup() {
  for (int i = 0; i < MAX_COVERS; i++) {
    sendState(i, First_CHILD_ID_COVER+i);
  }
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

  //böse
  //delay (300);
  if (currentTime - lastUpdateDisplay > DISPLAY_UPDATE_FREQUENCY) {
    lastUpdateDisplay = currentTime;
    displayTime();
  }


  // Only send values at a maximum frequency
  if (currentTime - lastSend > SEND_FREQUENCY) {
    lastSend = currentTime;
    uint16_t lux = lightSensor.readLightLevel();// Get Lux value
    if (lux != lastlux) {
      send(msgLux.set(lux));
      lastlux = lux;
    }
  send(msgRain.set(emergency));
  #ifdef MY_DEBUG_LOCAL
    Serial.print("lux:");
    Serial.println(lux);
  #endif
  }

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

  State[0]=mark.loop(button_mark_up, button_mark_down);
  State[1]=jal.loop(button_jal_up, button_jal_down);
//  mark.loop(button_mark_up, button_mark_down);
//  jal.loop(button_jal_up, button_jal_down);

  for (int i = 0; i < MAX_COVERS; i++) {
    if ( State[i] != oldState[i]||status[i] != oldStatus[i]) {
      sendState(i, First_CHILD_ID_COVER+i);
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
      if (val < 50 && State[message.sensor-First_CHILD_ID_COVER] != 2 && State[message.sensor-First_CHILD_ID_COVER] != 3) {
        //DOWN-Befehl einfügen
        bool button_mark_up=false;
        bool button_mark_down=true;
        mark.loop(button_mark_up, button_mark_down);

#ifdef MY_DEBUG_LOCAL
    Serial.print("GW Message down: ");
    Serial.println(val);
#endif
      }
      else if (val == 50) {
        //Stop-Befehle einfügen
        bool button_mark_up=false;
        bool button_mark_down=false;
        mark.loop(button_mark_up, button_mark_down);

#ifdef MY_DEBUG_LOCAL
    Serial.print("GW Message stop: ");
    Serial.println(val);
#endif
      }
      else if (val >50 && State[message.sensor-First_CHILD_ID_COVER] != 1 && State[message.sensor-First_CHILD_ID_COVER] != 4) {
       //UP-Befehle einfügen
       bool button_mark_up=true;
       bool button_mark_down=false;
       mark.loop(button_mark_up, button_mark_down);

#ifdef MY_DEBUG_LOCAL
    Serial.print("GW Msg up: ");
    Serial.println(val);
#endif
      }
    }

    //deaktiviert, da FHEM das ohne Änderung der .pm noch nicht senden dürfte...
    if (message.type == V_UP && State[message.sensor-First_CHILD_ID_COVER] != 1 && State[message.sensor-First_CHILD_ID_COVER] != 4) {
      // Set state to covering up and send it back to the gateway.
      State[message.sensor-First_CHILD_ID_COVER] = UP;
      bool button_mark_up=true;
      bool button_mark_down=false;
      mark.loop(button_mark_up, button_mark_down);
/*sendState();
      Serial.println("Moving cover 1 up.");

      // Activate actuator until the sensor returns HIGH in loop().
      digitalWrite(COVER2_ON_ACTUATOR_PIN, HIGH);*/
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
  //hier gehört die Doppelung für das 2. Coverr hin
  }
}

void readDS3231time(byte *second,
byte *minute,
byte *hour,
byte *dayOfWeek,
byte *dayOfMonth,
byte *month,
byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

void displayTime()
{
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,  &year);
  // send it to the serial monitor
  Serial.print(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  Serial.print(":");
  if (minute<10)
  {
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second<10)
  {
    Serial.print("0");
  }
  Serial.print(second, DEC);
  Serial.print(" ");
  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(month, DEC);
  Serial.print("/");
  Serial.println(year, DEC);
}
