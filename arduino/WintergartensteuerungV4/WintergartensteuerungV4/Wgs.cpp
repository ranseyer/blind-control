/*
  Wgs.h - Custom library for wintergarden.
  Created by Felix Neubauer, August 12, 2016.
*/


//#include "Arduino.h"
#include "Wgs.h"


const int STATE_UNKNOWN = 0;
const int STATE_ENABLED = 1;
const int STATE_DISABLING = 2;
const int STATE_DISABLED = 3;
const int STATE_ENABLING = 4;

Wgs::Wgs() {}

void Wgs::setPins(uint8_t pin_on, uint8_t pin_down, long duration)
{
  _pin_on = pin_on;
  _pin_down = pin_down;
  _duration = duration;
}
  
Wgs::Wgs(uint8_t pin_on, uint8_t pin_down, long duration)
{
  //pinMode(pin, OUTPUT);
  setPins(pin_on, pin_down, duration);
  _disable = false;
}


int Wgs::loop(bool button_disable, bool button_enable)
{

//debug("Loop. Button enable: "+button_enable);
//debug("Button disable: "+button_disable);
//debug("State: "+_state);

  if(_mute_time > millis()){
#ifdef MY_DEBUG_LOCAL
  debug("Muted");
#endif
    return _state;
  }
  
  if(_disable){
#ifdef MY_DEBUG_LOCAL
  debug("Detected rain!!!!!!!!!!!!!!!!!!!!!!!!!!");
#endif
    button_disable = true;
  }

  if(button_disable){
    if(_state == STATE_DISABLED || _state == STATE_DISABLING){ //Already disabled/disabling
#ifdef MY_DEBUG_LOCAL
    debug("Already disabling/disabled");
#endif
      return _state;
    }
    if(_state == STATE_ENABLING){
    #ifdef MY_DEBUG_LOCAL
    debug("Stop enabling");
    #endif
    _mute_time = millis() + 400;
      stopMovement(STATE_UNKNOWN);
      return _state;
    }
    #ifdef MY_DEBUG_LOCAL
    debug("Start disabling");
    #endif
    startMovement(STATE_DISABLING);

  }else if(button_enable){
    if(_state == STATE_ENABLED || _state == STATE_ENABLING){ //Already enabled/enabling
    #ifdef MY_DEBUG_LOCAL
    debug("Already enabling/enabled");
    #endif
      return _state;
    }
    if(_state == STATE_DISABLING){
    #ifdef MY_DEBUG_LOCAL
    debug("Stop disabling");
    #endif
    _mute_time = millis() + 400;
      stopMovement(STATE_UNKNOWN);
      return _state; //Welcher Rückgabecode?
    }
    #ifdef MY_DEBUG_LOCAL
    debug("Start enabling");
  #endif
    startMovement(STATE_ENABLING);
  }
  
  
  if (_finish_time <= millis() && _finish_time > 0) {
    #ifdef MY_DEBUG_LOCAL
   debug("reached finish time");
   #endif
    switch (_state) {
      case STATE_DISABLING:
        stopMovement(STATE_DISABLED);
        break;
      case STATE_ENABLING:
        stopMovement(STATE_ENABLED);
        break;
    }
  }
return _state;  
}

void Wgs::setDisable(boolean b)
{
  _disable = b;
}


void Wgs::stopMovement(int state) {
  _finish_time = 0; //Set destination time to 0 -> it's not active anymore
  digitalWrite(_pin_on, HIGH);
  delay(150);
  digitalWrite(_pin_down, HIGH);
  _state = state;
}


void Wgs::debug(String text) {
/*String prefix = String();
prefix = "["+ _pin_on;
prefix = prefix+" ";
prefix = prefix + _pin_down;
prefix = prefix +"] ";
String goal = String();
goal = prefix + text;*/

Serial.println(_pin_on + text);
}

void Wgs::startMovement(int state)
{
  setState(state);
  
  if(_state == STATE_DISABLING){ 
    digitalWrite(_pin_down, HIGH);//Activate relais and make it ready to disable this component
  }else if(_state == STATE_ENABLING){ 
    digitalWrite(_pin_down, LOW); //Activate relais and make it ready to enable this component
  }
  
  delay(150);
  digitalWrite(_pin_on, LOW); //Activate motor
  _finish_time = millis() + _duration; //Set destination time
}

void Wgs::setState(int i)  //-1 = unknown. 0 = enabled; 1 = move_disable; 2 = disabled; 3 = move_enable;
{
  _state = i;
}

int Wgs::getState()
{
  return _state;
}

