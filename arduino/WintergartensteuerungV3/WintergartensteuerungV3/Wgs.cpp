/*
  Wgs.h - Custom library for wintergarden.
  Created by Felix Neubauer, August 12, 2016.
*/


#include "Arduino.h"
#include "Wgs.h"


const int STATE_UNKNOWN = 0;
const int STATE_ENABLED = 1;
const int STATE_DISABLING = 2;
const int STATE_DISABLED = 3;
const int STATE_ENABLING = 4;

Wgs::Wgs(int pin_on, int pin_down, long duration)
{
	//pinMode(pin, OUTPUT);
	_pin_on = pin_on;
	_pin_down = pin_down;
	_duration = duration;
_disable = false;
}


void Wgs::loop(bool button_disable, bool button_enable)
{


//debug("Loop. Button enable: "+button_enable);
//debug("Button disable: "+button_disable);
//debug("State: "+_state);

	if(_mute_time > millis()){
debug("Muted");
		return;
	}
	
	if(_disable){
debug("Detected rain!!!!!!!!!!!!!!!!!!!!!!!!!!");
		button_disable = true;
	}

	if(button_disable){
		if(_state == STATE_DISABLED || _state == STATE_DISABLING){ //Already disabled/disabling
    debug("Already disabling/disabled");
			return;
		}
		if(_state == STATE_ENABLING){
    debug("Stop enabling");
		_mute_time = millis() + 400;
			stopMovement(STATE_UNKNOWN);
			return;
		}
		
    debug("Start disabling");
		startMovement(STATE_DISABLING);

	}else if(button_enable){
		if(_state == STATE_ENABLED || _state == STATE_ENABLING){ //Already enabled/enabling
    debug("Already enabling/enabled");
			return;
		}
		if(_state == STATE_DISABLING){
    debug("Stop disabling");
		_mute_time = millis() + 400;
			stopMovement(STATE_UNKNOWN);
			return;
		}
		
    debug("Start enabling");
		startMovement(STATE_ENABLING);
	}
	
	
  if (_finish_time <= millis() && _finish_time > 0) {
   debug("reached finish time");
    switch (_state) {
      case STATE_DISABLING:
        stopMovement(STATE_DISABLED);
        break;
      case STATE_ENABLING:
        stopMovement(STATE_ENABLED);
        break;
    }
  }
	
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
