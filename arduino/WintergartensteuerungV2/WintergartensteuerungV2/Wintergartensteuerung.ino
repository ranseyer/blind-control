// (C) www.neuby.de
//V2.005
// Enable debug prints to serial monitor
#define MY_DEBUG
// Enable RS485 transport layer
#define MY_RS485
// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 2
// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600
#define MY_NODE_ID 118
#define MY_TRANSPORT_WAIT_READY_MS 3000
#include <MySensors.h>
#include <Arduino.h>
#include <SPI.h>
#include <Bounce2.h>
#include <Wgs.h>
// For RTC
#include "Wire.h"
#define DS3231_I2C_ADDRESS 0x68

#define CHILD_ID_LIGHT 0
#define CHILD_ID_Rain 1   // Id of the sensor child
#define CHILD_ID_MARKDWN 2
#define CHILD_ID_MARKUP 3
#define CHILD_ID_MARKRAIN 4
#define CHILD_ID_JALDWN 5
#define CHILD_ID_JALUP 6
#define CHILD_ID_CONFIG1 102 // Id for Jal-settings
//CHILD_ID_SERVO

unsigned long SEND_FREQUENCY = 180000; // Sleep time between reads (in milliseconds)

// Initialize motion message
MyMessage msgRain(CHILD_ID_Rain, V_RAIN);
#define DIGITAL_INPUT_SENSOR 3   // The digital input you attached your rain sensor.  (Only 2 and 3 generates interrupt!)
#define SENSOR_INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)

MyMessage msgMarkise1(CHILD_ID_MARKISE1, V_DIMMER);



// Buttons
Bounce debounceJalUp    = Bounce();
Bounce debounceJalDown  = Bounce();
Bounce debounceMarkEmergency  = Bounce();
Bounce debounceMarkUp    = Bounce();
Bounce debounceMarkDown  = Bounce();

#define BT_PRESS_None             0                   //
#define BT_PRESS_JalUp            1                   //
#define BT_PRESS_JalDown          2                   //
#define BT_PRESS_Stop             3                   //
#define BT_PRESS_MarkUp           4                   //
#define BT_PRESS_MarkDown         5                   //
#define BT_PRESS_MarkEmergency    6                   //



//unsigned long SEND_FREQUENCY =    20000; // Minimum time between send (in milliseconds). We don't wnat to spam the gateway.
double ppwh = ((double)PULSE_FACTOR)/1000; // Pulses per watt hour
bool pcReceived = false;
volatile unsigned long pulseCount = 0;
volatile unsigned long lastBlink = 0;
volatile unsigned long watt = 0;
unsigned long oldPulseCount = 0;
unsigned long oldWatt = 0;
double oldKwh;
unsigned long lastSend;

int timeOfLastChange = 8;

// Input Pins for Switch Markise Up/Down
const int SwMarkUp = 8;
const int SwMarkDown = 9;
// Input Pins for Switch Jalosie Up/Down
const int SwJalUp = 7;
const int SwJalDown = 6;
//Notfall
const int SwEmergency = 5;


// Output Pins
const int JalUp = 10;
const int JalDown = 11;
//const int JalRevers = 12;
const int MarkUp = 13;
const int MarkDown = 14;

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

boolean MDown = false;
boolean MUp = false;

Wgs mark(MarkUp, MarkDown, 55000);
Wgs jal(JalUp, JalDown, 65000);

byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

void before() {

  // initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion)
  pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);
  digitalWrite(DIGITAL_INPUT_SENSOR, HIGH);
  pulseCount = oldPulseCount = 0;
  attachInterrupt(SENSOR_INTERRUPT, onPulse, FALLING);

  // Initialize In-/Outputs
  pinMode(SwMarkUp, INPUT_PULLUP);
  pinMode(SwMarkDown, INPUT_PULLUP);
  pinMode(SwJalUp, INPUT_PULLUP);
  pinMode(SwJalDown, INPUT_PULLUP);
  pinMode(SwEmergency, INPUT_PULLUP);
  pinMode(MarkUp, OUTPUT);
  pinMode(MarkDown, OUTPUT);
  pinMode(JalUp, OUTPUT);
  pinMode(JalDown, OUTPUT);
  digitalWrite(MarkUp, HIGH);
  digitalWrite(MarkDown, HIGH);
  digitalWrite(JalUp, HIGH);
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
  debounceMarkEmergency.interval(5);

  Wire.begin();
}

void setup()
{

  //  Serial.begin(57600);

}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Light Rain Sensor", "0.1.0");

  // Register all sensors to gateway (they will be created as child devices)
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  present(CHILD_ID_Rain, S_RAIN);
  present(CHILD_ID_MARKISE1, S_COVER);
  present(CHILD_ID_CONFIG1, S_CUSTOM);


}





void loop()
{
//Serial.print("Loop/");
    #ifdef MY_OTA_FIRMWARE_FEATURE
    #endif
unsigned long currentTime = millis();

  // Only send values at a maximum frequency or woken up from sleep
  if (currentTime - lastSend > SEND_FREQUENCY)
  {
    lastSend = currentTime;
    if (flow != oldflow) {
      oldflow = flow;
#ifdef MY_DEBUG_LOCAL
      Serial.print("l/min:");
      Serial.println(flow);
#endif
      // Check that we dont get unresonable large flow value.
      // could hapen when long wraps or false interrupt triggered
      if (flow < ((unsigned long)MAX_FLOW)) {
        send(flowMsg.set(flow, 2));                   // Send flow value to gw
      }
    }

    // No Pulse count received in 2min
    if (currentTime - lastPulse > 120000) {
      flow = 0;
    }

    // Pulse count has changed
    if (pulseCount != oldPulseCount) {
      oldPulseCount = pulseCount;
#ifdef MY_DEBUG_LOCAL
      Serial.print("pulsecnt:");
      Serial.println(pulseCount);
#endif
      send(lastCounterMsg.set(pulseCount));                  // Send  pulsecount value to gw in VAR1

      double volume = ((double)pulseCount / ((double)PULSE_FACTOR));
      if (volume != oldvolume) {
        oldvolume = volume;
#ifdef MY_DEBUG_LOCAL
        Serial.print("vol:");
        Serial.println(volume, 3);
#endif
        send(volumeMsg.set(volume, 3));               // Send volume value to gw
      }
    }
  }

    // Read buttons, interface
    uint8_t buttonPressed = 0;
    buttonPressed = processButtons();
    switch (buttonPressed) {
      case BT_PRESS_MarkUp:
////////////////////
        //setPosition(100);
        MUp=true;
        MDown=false;
        //send(msgUp.set(1), 1);
        mark.loop(MUp, MDown);
        Serial.print("Mup/");
        break;

      case BT_PRESS_MarkDown:
        //setPosition(0);
        MDown=true;
        MUp=false;
        mark.loop(MUp, MDown);
        //send(msgDown.set(1), 1);
        Serial.print("MDown/");
        break;

  //    case BT_PRESS_Stop:
  //      ShutterStop();
  //      //send(msgStop.set(1), 1);
  //      break;
    }




  // Read digital motion value
  bool tripped = digitalRead(DIGITAL_INPUT_SENSOR) == HIGH;
  Serial.println(tripped);
  send(msgRain.set(tripped?"0":"1"));  // Send tripped value to gw
  displayTime();


//  bool button_mark_up = digitalRead(SwMarkUp) == LOW;
//  bool button_mark_down = digitalRead(SwMarkDown) == LOW;
//  bool button_jal_up = digitalRead(SwJalUp) == LOW;
//  bool button_jal_down = digitalRead(SwJalDown) == LOW;
bool emergency = digitalRead(SwEmergency) == LOW; //Current use: in case of rain

//wieder rein !!!!!
mark.setDisable(emergency);

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
        MDown = true;
        autostart_done = true;
      }
    }

  }


  mark.loop(MUp, MDown);
  //jal.loop(button_jal_up, button_jal_down);



}


void receive(const MyMessage & message) {
  if (message.sensor == CHILD_ID_MARKDWN) {
//    myservo.attach(SERVO_DIGITAL_OUT_PIN);
//    attachedServo = true;
    if (message.isAck()) {
      Serial.println("Ack from gw rec.");
    }
    if (message.type == V_DIMMER) { // This could be M_ACK_VARIABLE or M_SET_VARIABLE
      int val = message.getInt();
      if (val > 3) {
        val = 3;
      };
      MUp=true;
      MDown=false;
      mark.loop(MUp, MDown); // meine Lib um geregelt herauf / runterzufahren !
      Serial.print("Mup/");
      //myservo.write(SERVO_MAX + (SERVO_MIN - SERVO_MAX) / 100 * val); // sets the servo position 0-180
      //myservo.write(SERVO_MAX + (SERVO_MIN - SERVO_MAX) / 100 * (130 - val * 40)); // sets the servo position 0-180
      // Write some debug info
      //Serial.print("Servo change; state: ");
      //Serial.println(val);
    }
    timeOfLastChange = millis();
  }
  else if (message.sensor == CHILD_ID_Rain) {
    if (message.type == V_VAR1) {
      pulseCount = message.getULong();
      flow = oldflow = 0;
      //Serial.print("Rec. last pulse count from gw:");
      //Serial.println(pulseCount);
      pcReceived = true;
    }
  }
  else if (message.sensor == CHILD_ID_RELAY) {

    if (message.type == V_LIGHT) {
      // Change relay state
      state = message.getBool();
      digitalWrite(RELAY_PIN, state ? RELAY_ON : RELAY_OFF);
#ifdef MY_DEBUG
      // Write some debug info
      Serial.print("Gw change relay:");
      Serial.print(message.sensor);
      Serial.print(", New status: ");
      Serial.println(message.getBool());
#endif
    }
  }
  else if (message.sensor == CHILD_ID_CONFIG) {
    if (message.type == V_VAR1) {
      int tempMaxPump = message.getInt(); //upper temp level at warmwater circle pump
    }
    else if (message.type == V_VAR2) {
      int tempMaxHeatingPump = message.getInt(); //temperature to switch internal heating pump to highest level
    }
    else if (message.type == V_VAR3) {
      int tempLowExtToLevelIII = message.getInt(); //External low temperature to switch internal heating pump to highest level
    }
    else if (message.type == V_VAR4) {
      int tempLowExtToLevelII = message.getInt(); //External highest temperature to switch internal heating pump to medium level
    }
    else if (message.type == V_VAR5) {
      if (message.getBool()) {
        lastPumpSwitch = millis(); //if true: Reset timer (Heartbeat functionality)
#ifdef MY_DEBUG_LOCAL
        // Write some debug info
        Serial.print("Timer reset to ");
        Serial.print(lastPumpSwitch);
#endif
      } else {
        lastPumpSwitch = 0; //if false: switch immediately if necessary
#ifdef MY_DEBUG_LOCAL
        // Write some debug info
        Serial.print("Timer deleted, lastPumpSwitch=");
        Serial.print(lastPumpSwitch);
#endif
      }
    }
  }
  else if (message.sensor == CHILD_ID_CONFIG0) {
    if (message.type == V_VAR1) {
      autoMode = message.getBool(); //enable autoMode
    }
  }
}


/*
Interrupt-Routine
*/
void onPulse()
{
    unsigned long newBlink = micros();
    unsigned long interval = newBlink - lastBlink;

    if (interval != 0)
    {
      lastPulse = millis();
      if (interval < 1000000L) {
        // Sometimes we get interrupt on RISING,  1000000 = 1sek debounce ( max 60 l/min)
        return;
      }
      flow = (60000000.0 / interval) / ppl;
    }
    lastBlink = newBlink;
   pulseCount++;
}