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
#define MY_TRANSPORT_WAIT_READY_MS 3000
#include <Arduino.h>
#include <SPI.h>
#include <Bounce2.h>
//#include <Wgs.h> //WintergartenLiB
#include "Wire.h" // For RTC
#define DS3231_I2C_ADDRESS 0x68 // For RTC
#include <MySensors.h>
#define SN "DoubleCover"
#define SV "0.0.1"

// Actuators for moving the cover up and down respectively.
#define COVER_UP_ACTUATOR_PIN 2
#define COVER_DOWN_ACTUATOR_PIN 3
#define COVER2_UP_ACTUATOR_PIN 2
#define COVER2_DOWN_ACTUATOR_PIN 3

// Sensors for finding out when the cover has reached its up/down position.
// These could be simple buttons or linear hall sensors.
#define COVER_UP_SENSOR_PIN 4
#define COVER_DOWN_SENSOR_PIN 5
#define COVER2_UP_SENSOR_PIN 4
#define COVER2_DOWN_SENSOR_PIN 5

#define CHILD_ID_LIGHT 0
#define CHILD_ID 1
#define CHILD2_ID 2
#define CHILD_ID_Rain 3   // Id of the sensor child
#define CHILD_ID_CONFIG_COVER 102
#define CHILD2_ID_CONFIG_COVER 103


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

MyMessage upMessage(CHILD_ID, V_UP);
MyMessage downMessage(CHILD_ID, V_DOWN);
MyMessage stopMessage(CHILD_ID, V_STOP);
MyMessage statusMessage(CHILD_ID, V_STATUS);
MyMessage upMessage2(CHILD2_ID, V_UP);  /// V_UP ???
MyMessage downMessage2(CHILD2_ID, V_DOWN);
MyMessage stopMessage2(CHILD2_ID, V_STOP);
MyMessage statusMessage2(CHILD2_ID, V_STATUS);

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

void setup() {
  pinMode(COVER_UP_SENSOR_PIN, INPUT);
  pinMode(COVER_DOWN_SENSOR_PIN, INPUT);
  pinMode(COVER2_UP_SENSOR_PIN, INPUT);
  pinMode(COVER2_DOWN_SENSOR_PIN, INPUT);
}

void presentation() {
  sendSketchInfo(SN, SV);
  present(CHILD_ID, S_COVER);
  present(CHILD2_ID, S_COVER);
  present(CHILD_ID_CONFIG_COVER, S_CUSTOM);
  present(CHILD2_ID_CONFIG_COVER, S_CUSTOM);
}

void loop() {
  //if (!initial_state_sent) {
  //  sendState();
  //  initial_state_sent = true;
  //}

  if (state == IDLE) {
    digitalWrite(COVER_UP_ACTUATOR_PIN, LOW);
    digitalWrite(COVER_DOWN_ACTUATOR_PIN, LOW);
    digitalWrite(COVER2_UP_ACTUATOR_PIN, LOW);
    digitalWrite(COVER2_DOWN_ACTUATOR_PIN, LOW);
  }




///// verdoppeln !  (StatusSchalter habe ich nicht !)

  if (state == UP && digitalRead(COVER_UP_SENSOR_PIN) == HIGH) {
    Serial.println("Cover is up.");
    // Update status and state; send it to the gateway.
    status = 1;
    state = IDLE;
    sendState();
    // Actuators will be disabled in next loop() iteration.
  }

  if (state == DOWN && digitalRead(COVER_DOWN_SENSOR_PIN) == HIGH) {
    Serial.println("Cover is down.");
    // Update status and state; send it to the gateway.
    status = 0;
    state = IDLE;
    sendState();
    // Actuators will be disabled in next loop() iteration.
  }


//verdoppeln




}

//Hier muesste man entscheiden um welches Child es geht beim doppeln...


void receive(const MyMessage &message) {
  if (message.type == V_UP) {
    // Set state to covering up and send it back to the gateway.
    state = UP;
    sendState();
    Serial.println("Moving cover up.");

    // Activate actuator until the sensor returns HIGH in loop().
    digitalWrite(COVER_UP_ACTUATOR_PIN, HIGH);
  }

  if (message.type == V_DOWN) {
    // Set state to covering up and send it back to the gateway.
    state = DOWN;
    sendState();
    Serial.println("Moving cover down.");
    // Activate actuator until the sensor returns HIGH in loop().
    digitalWrite(COVER_DOWN_ACTUATOR_PIN, HIGH);
  }

  if (message.type == V_STOP) {
    // Set state to idle and send it back to the gateway.
    state = IDLE;
    sendState();
    Serial.println("Stopping cover.");

    // Actuators will be switched off in loop().
  }
}
