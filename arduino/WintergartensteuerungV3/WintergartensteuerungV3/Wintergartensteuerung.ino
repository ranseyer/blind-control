/*
 * Changelog: Kommentare zum weiteren Vorgehen eingefügt
 * Die Zuordnung der Relais habe ich nicht verstanden,
 * wo das zu korrigieren ist
 */

#define SN "DoubleCover"
#define SV "0.2.2" //Das darf jetzt ruhig mal sein

//#define MY_DEBUG
//#define MY_DEBUG_LOCAL //Für lokale Debug-Ausgaben
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
#include "Wgs.h"
// For RTC
#include "Wire.h" //warum andere Schreibweise ?
#include <TimeLib.h>
#include <DS3232RTC.h>
#define DS3231_I2C_ADDRESS 0x68 //könnte man evtl. umstellen, auf die RTC-lib
//#include <LiquidCrystal_I2C.h>
#include <MySensors.h>

//für die millis()-Berechnung, wann wieder gesendet werden soll
unsigned long SEND_FREQUENCY = 180000; // Sleep time between reads (in milliseconds)
unsigned long lastSend = 0;
unsigned long DISPLAY_UPDATE_FREQUENCY = 300; // (in milliseconds)
unsigned long lastUpdateDisplay = 0;

/*für RTC; code ist von hier: https://www.mysensors.org/build/display
Da steht auch, wie man ein I2C-Display ansteuert...
und die Zeit vom Controller holt
*/
bool timeReceived = false;
unsigned long lastUpdate=0, lastRequest=0;

// Initialize display. Google the correct settings for your display.
// The follwoing setting should work for the recommended display in the MySensors "shop".
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //Funktionierte leider beim compilieren nicht, daher deaktiviert
//LiquidCrystal_I2C lcd(1);

#define CHILD_ID_LIGHT 0
#define CHILD_ID_RAIN 1   // Id of the sensor child
#define First_CHILD_ID_COVER 2
#define MAX_COVERS 2

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
  //Serial.begin(57600);

  setSyncProvider(RTC.get);

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
  // Request latest time from controller at startup
  requestTime();
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

//  //Könnte auch weg, wenn kein Display mehr angeschlossen werden soll
//  if (currentTime - lastUpdateDisplay > DISPLAY_UPDATE_FREQUENCY) {
//    lastUpdateDisplay = currentTime;
//    displayTime();
//    //updateDisplay();
//  }
  // If no time has been received yet, request it every 10 second from controller
  // When time has been received, request update every hour
  if ((!timeReceived && (currentTime-lastRequest) > (10UL*1000UL))
    || (timeReceived && (currentTime-lastRequest) > (60UL*1000UL*60UL))) {
    // Request time from controller.
#ifdef MY_DEBUG_LOCAL
    Serial.println("requesting time");
#endif
    requestTime();
    lastRequest = currentTime;
  }

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

void receiveTime(unsigned long controllerTime) {
  // Ok, set incoming time
 #ifdef MY_DEBUG_LOCAL
  Serial.print("Time value received: ");
  Serial.println(controllerTime);
#endif
  RTC.set(controllerTime); // this sets the RTC to the time from controller - which we do want periodically
  timeReceived = true;
}

/*void updateDisplay(){
  tmElements_t tm;
  RTC.read(tm);
  // Print date and time
  lcd.home();
  lcd.print(tm.Day);
  lcd.print("/");
  lcd.print(tm.Month);
//  lcd.print(" ");
//  lcd.print(tmYearToCalendar(tm.Year)-2000);
  lcd.print(" ");
  printDigits(tm.Hour);
  lcd.print(":");
  printDigits(tm.Minute);
  lcd.print(":");
  printDigits(tm.Second);
  // Go to next line and print temperature
  lcd.setCursor ( 0, 1 );
  lcd.print("Temp: ");
  lcd.print(RTC.temperature()/4);
  lcd.write(223); // Degree-sign
  lcd.print("C");
}
void printDigits(int digits){
  if(digits < 10)
    lcd.print('0');
  lcd.print(digits);
}*/
