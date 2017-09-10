/*
  Wgs.h - Custom library for wintergarden.
  Created by Felix Neubauer, August 12, 2016.
*/

#ifndef Wgs_h
#define Wgs_h

#include "Arduino.h"

class Wgs
{
  public:
    Wgs();
    void setPins(uint8_t pin_on, uint8_t pin_down, long duration);
    Wgs(uint8_t pin_on, uint8_t pin_down, long duration);
    int loop(bool button_disable, bool button_enable);
    int getState();
    void setDisable(boolean b);
    void startMovement(int state);
    void stopMovement(int state);
    void debug(String text);
    void setState(int state); //-1 = unknown. 0-1 = state with 0 = disabled and 1 = completely enabled
  private:
    int _pin_on;
    int _pin_down;
  long _duration;
  bool _disable;
  int _state;
  long _finish_time;
  long _mute_time;
};

#endif

