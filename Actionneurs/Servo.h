#ifndef SERVO_H
#define SERVO_H

#include "mbed.h"

class Servo {

public:
    // Constructeur:
    Servo(PinName Pin);
    
    // Déplacement:
    void pos(int pos);
    void enable(int startPos, float period);
    void disable();

private:
    // Gestion de l'impulsion:
    void startPulse();
    void endPulse();

    int m_position;
    DigitalOut m_pin;
    Ticker m_pulse;
    Timeout m_pulseStop;
};

#endif