/*

OK, jetzt habe ich mir das mit der WG-lib nochmal näher angesehen.
Es wäre m.E. besser, die lib zu aktivieren, dann aber dort noch einige Anpassungen zu machen:

Es braucht einen bzw. mehrere Rückgabewerte (am besten einen STATE, die Info, ob der Controller zu informieren ist (Senden?: bool) und optimalerweise noch einen Prozentwert, wo das Wgs-Device grade ist. Das könnte man dann nach der jeweiligen wgs.loop() in der Hauptloop() verarbeiten.
Für die Zukunft wäre noch ein %-Wert als Zielangabe für das Device klasse, dazu wären aber dort weitere Umbauten erforderlich
Wenn ich es richtig verstanden habe, gibt es dort einen Fahrbefehl, den man direkt und gefahrlos aufrufen kann startMovement(x), auf die Schnelle konnte ich nur nicht rausfinden, was hoch und runter ist, dto. für stopMovement(). Das wäre in den receive()-Teil reinzuarbeiten
*/



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
    Wgs(int pin_on, int pin_down, long duration);
    void loop(bool button_disable, bool button_enable);
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
