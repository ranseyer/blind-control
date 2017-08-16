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
    float getState();
    void setDisable(boolean b);
    void startMovement(int state);
    void stopMovement(int state);
    void debug(String text);
    void setState(float state); //0 = Komplett eingefahren. 1.00f = Komplett ausgefahren. 
  private:
    int _pin_on;
    int _pin_down;
  long _duration;
  bool _disable;
  float _state; //0 = Komplett eingefahren. 1.00f = Komplett ausgefahren. 
  long _finish_time;
  long _mute_time;
};

#endif

