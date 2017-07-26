// (C) www.neuby.de
//V3.001
// Enable debug prints to serial monitor
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
#include <Bounce2.h>
//#include <Wgs.h> //WintergartenLiB
#include "Wire.h" // For RTC
#define DS3231_I2C_ADDRESS 0x68 // For RTC
#include <MySensors.h>
#define SN "DoubleCover"
#define SV "0.0.1"
//für die millis()-Berechnung, wann wieder gesendet werden soll
unsigned long SEND_FREQUENCY = 180000; // Sleep time between reads (in milliseconds)
unsigned long lastSend = 0;

// Actuators for moving the cover up and down respectively.
#define COVER1_ON_ACTUATOR_PIN 10 // Jal (default)
#define COVER1_DOWN_ACTUATOR_PIN 11
#define COVER2_ON_ACTUATOR_PIN 13 // Mark (default#)
#define COVER2_DOWN_ACTUATOR_PIN 14

// Sensors for finding out when the cover has reached its up/down position.
// These could be simple buttons or linear hall sensors.
#define COVER1_UP_SW_PIN 7 // Jal
#define COVER1_DOWN_SW_PIN 6
#define COVER2_UP_SW_PIN 8 //Mark
#define COVER2_DOWN_SW_PIN 9
#define EMERGENCY_SW_PIN 5

// Initialize motion message
#define DIGITAL_INPUT_SENSOR 3   // The digital input you attached your rain sensor.  (Only 2 and 3 generates interrupt!)
#define SENSOR_INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)


#define CHILD_ID_LIGHT 0
#define CHILDCOVER1_ID 1
#define CHILDCOVER2_ID 2
#define CHILD_ID_RAIN 3   // Id of the sensor child
#define CHILDCOVER1_ID_CONFIG_COVER 102
#define CHILDCOVER2_ID_CONFIG_COVER 103


// Buttons
Bounce debounceCover1Up    = Bounce();
Bounce debounceCover1Down  = Bounce();
Bounce debounceEmergency  = Bounce();
Bounce debounceCover2Up    = Bounce();
Bounce debounceCover2Down  = Bounce();
#define BT_PRESS_None               0                   //
#define BT_PRESS_Cover1Up           1                   //
#define BT_PRESS_Cover1Down         2                   //
#define BT_PRESS_Stop             3                   //
#define BT_PRESS_Cover2Up           4                   //
#define BT_PRESS_Cover2Down         5                   //
#define BT_PRESS_Emergency          6                   //



// Internal representation of the cover state.
enum State {
  IDLE,
  UP, // Window covering. Up.
  DOWN, // Window covering. Down.
};
enum State2 {
  IDLE2,
  UP2, // Window covering. Up.
  DOWN2, // Window covering. Down.
};

static int state = IDLE;
static int status = 0; // 0=cover is down, 1=cover is up
static bool initial_state_sent = false;
static int State2 = IDLE;
static int status2 = 0; // 0=cover is down, 1=cover is up
static bool initial_state_sent2 = false;

MyMessage upMessage(CHILDCOVER1_ID, V_UP);
MyMessage downMessage(CHILDCOVER1_ID, V_DOWN);
MyMessage stopMessage(CHILDCOVER1_ID, V_STOP);
MyMessage statusMessage(CHILDCOVER1_ID, V_STATUS);
MyMessage upMessage2(CHILDCOVER2_ID, V_UP);  /// V_UP ???
MyMessage downMessage2(CHILDCOVER2_ID, V_DOWN);
MyMessage stopMessage2(CHILDCOVER2_ID, V_STOP);
MyMessage statusMessage2(CHILDCOVER2_ID, V_STATUS);
MyMessage msgRain(CHILD_ID_RAIN, V_RAIN);
MyMessage msgLight(CHILD_ID_LIGHT, V_LIGHT_LEVEL);


//noch vereinfachen...
void sendState() {
  // Send current state and status to gateway.
  send(upMessage.set(state == UP));
  send(downMessage.set(state == DOWN));
  send(stopMessage.set(state == IDLE));
  send(statusMessage.set(status));
  send(upMessage2.set(state == UP2));
  send(downMessage2.set(state == DOWN2));
  send(stopMessage2.set(state == IDLE2));
  send(statusMessage2.set(status2));

}


void before() {
  // Initialize In-/Outputs
  pinMode(COVER1_UP_SW_PIN, INPUT_PULLUP);
  pinMode(COVER1_DOWN_SW_PIN, INPUT_PULLUP);
  pinMode(COVER2_UP_SW_PIN, INPUT_PULLUP);
  pinMode(COVER2_DOWN_SW_PIN, INPUT_PULLUP);
  pinMode(EMERGENCY_SW_PIN, INPUT_PULLUP);

  // initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion)
  pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);
  digitalWrite(DIGITAL_INPUT_SENSOR, HIGH);
  //pulseCount = oldPulseCount = 0;
  //attachInterrupt(SENSOR_INTERRUPT, onPulse, CHANGE); //Unterstellt, es soll nur ein ja/nein-Signal sein

  pinMode(COVER1_ON_ACTUATOR_PIN, OUTPUT);
  pinMode(COVER2_ON_ACTUATOR_PIN, OUTPUT);
  pinMode(COVER1_DOWN_ACTUATOR_PIN, OUTPUT);
  pinMode(COVER2_DOWN_ACTUATOR_PIN, OUTPUT);
  digitalWrite(COVER1_ON_ACTUATOR_PIN, HIGH);
  digitalWrite(COVER2_ON_ACTUATOR_PIN, HIGH);
  digitalWrite(COVER1_DOWN_ACTUATOR_PIN, HIGH);
  digitalWrite(COVER2_DOWN_ACTUATOR_PIN, HIGH);


  // After setting up the button, setup debouncer
  debounceCover1Up.attach(COVER1_UP_SW_PIN);
  debounceCover1Up.interval(5);
  debounceCover1Down.attach(COVER1_DOWN_SW_PIN);
  debounceCover1Down.interval(5);
  debounceCover2Up.attach(COVER2_UP_SW_PIN);
  debounceCover2Up.interval(5);
  debounceCover2Down.attach(COVER2_DOWN_SW_PIN);
  debounceCover2Down.interval(5);
  debounceEmergency.attach(EMERGENCY_SW_PIN);
  debounceEmergency.interval(5);
  Wire.begin();
}






void setup() {
}

void presentation() {
  sendSketchInfo(SN, SV);
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  present(CHILDCOVER1_ID, S_COVER);
  present(CHILDCOVER2_ID, S_COVER);
  present(CHILD_ID_RAIN, S_RAIN);
  present(CHILDCOVER1_ID_CONFIG_COVER, S_CUSTOM);
  present(CHILDCOVER2_ID_CONFIG_COVER, S_CUSTOM);

}

void loop() {
//Serial.println("V0.0.1 test");

  if (state == IDLE) {
    digitalWrite(COVER1_ON_ACTUATOR_PIN, HIGH);
    digitalWrite(COVER1_DOWN_ACTUATOR_PIN, HIGH);
    digitalWrite(COVER2_ON_ACTUATOR_PIN, HIGH);
    digitalWrite(COVER2_DOWN_ACTUATOR_PIN, HIGH);
  }


  //Serial.print("Loop/");
  unsigned long currentTime = millis();

    // Only send values at a maximum frequency or woken up from sleep
    if (currentTime - lastSend > SEND_FREQUENCY)
    {
      lastSend = currentTime;
  /*Hier einfügen:
  Miss und Sende den aktuellen Lichtlevel
  Miss und sende den aktuellen Regenstatus (sofern nicht nur Digital ja/nein); das wäre in der ISR aufgehoben bzw. */
  #ifdef MY_DEBUG_LOCAL
    //    Serial.print("l/min:");
    //    Serial.println(flow);
  #endif
    }

      // Read buttons, interface
      uint8_t buttonPressed = 0;
      buttonPressed = processButtons();
      switch (buttonPressed) {
        case BT_PRESS_Cover1Up:
          //setPosition(100);
          //send(msgUp.setSensor(CHILD_ID_COVER1).set(1)); //sollte wohl nicht mit ACK-Anforderung gesendet werden
          digitalWrite(COVER1_ON_ACTUATOR_PIN, HIGH);
          /*          MUp=true;
          MDown=false;
          mark.loop(MUp, MDown);*/
          #ifdef MY_DEBUG_LOCAL
          Serial.println("C1up");
          #endif
          break;

        case BT_PRESS_Cover1Down:
          //setPosition(0);
/*          MDown=true;
          MUp=false;
          mark.loop(MUp, MDown);
          digitalWrite(COVER1_ON_ACTUATOR_PIN, HIGH);
          digitalWrite(COVER1_DOWN_ACTUATOR_PIN, HIGH); */

          //send(msgDown.set(1), 1);
          Serial.println("C1down");
          break;


         case BT_PRESS_None:
         state = IDLE;
         Serial.println("NoButton");
         break;

    //    case BT_PRESS_Stop:
    //      ShutterStop();
    //      //send(msgStop.set(1), 1);
    //      break;
  }  //End of button loop



    /* Read digital motion value
    bool tripped = digitalRead(DIGITAL_INPUT_SENSOR) == HIGH;
    Serial.println(tripped);
    send(msgRain.set(tripped?"0":"1"));  // Send tripped value to gw

    //displayTime();
*/

  //  bool button_mark_up = digitalRead(SwMarkUp) == LOW;
  //  bool button_mark_down = digitalRead(SwMarkDown) == LOW;
  //  bool button_jal_up = digitalRead(SwJalUp) == LOW;
  //  bool button_jal_down = digitalRead(SwJalDown) == LOW;
  bool emergency = digitalRead(EMERGENCY_SW_PIN) == LOW; //Current use: in case of rain

  delay(500);                  // waits

}

void receive(const MyMessage &message) {

  if (message.sensor == CHILDCOVER1_ID) {
      if (message.isAck()) {
      Serial.println("Ack child1 from gw rec.");
    }
    if (message.type == V_UP) {
      // Set state to covering up and send it back to the gateway.
      state = UP;
      sendState();
      Serial.println("Moving cover 1 up.");

      // Activate actuator until the sensor returns HIGH in loop().
      digitalWrite(COVER2_ON_ACTUATOR_PIN, HIGH);
    }
    if (message.type == V_DOWN) {
      // Set state to covering up and send it back to the gateway.
      state = DOWN;
      sendState();
      Serial.println("Moving cover 1 down.");

      // Activate actuator until the sensor returns HIGH in loop().
      digitalWrite(COVER2_DOWN_ACTUATOR_PIN, HIGH);
    }
   }
  if (message.sensor == CHILDCOVER2_ID) {
      if (message.isAck()) {
      Serial.println("Ack child2 from gw rec.");
    }
    if (message.type == V_UP) {
      // Set state to covering up and send it back to the gateway.
      state = UP2;
      sendState();
      Serial.println("Moving cover 2 up.");

      // Activate actuator until the sensor returns HIGH in loop().
      digitalWrite(COVER2_ON_ACTUATOR_PIN, HIGH);
    }
    if (message.type == V_DOWN) {
      // Set state to covering up and send it back to the gateway.
      state = DOWN2;
      sendState();
      Serial.println("Moving cover 2 down.");

      // Activate actuator until the sensor returns HIGH in loop().
      digitalWrite(COVER2_DOWN_ACTUATOR_PIN, HIGH);
    }

    //nextimeOfLastChange = millis();
  }



  if (message.type == V_STOP) {
    // Set state to idle and send it back to the gateway.
    state = IDLE;
    sendState();
    Serial.println("Stopping cover.");

    // Actuators will be switched off in loop().
  }
}



// Buttons auswerten
uint8_t processButtons() {
  uint8_t result = BT_PRESS_None;
  if (debounceCover1Up.update()) {
    // Button Up change detected
    if (debounceCover1Up.fell()) {
      result = BT_PRESS_Cover1Up;
      Serial.println("Diag C1Up");
    }
    else result = BT_PRESS_Stop; // Button released
    Serial.println("Diag Stop");
  }

  if (debounceCover1Down.update()) {
    // Button Down change detected
    if (debounceCover1Down.fell()){
      result = BT_PRESS_Cover1Down;
      Serial.println("Diag C1Dwn");
    }
    else result = BT_PRESS_Stop;
  }

/*  if (debounceStop.update()) {
    // Button Stop change detected
    if (debounceStop.fell()) result = BT_PRESS_STOP;
  } */

  return result;
}
