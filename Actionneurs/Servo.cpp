#include "Servo.h"
#include "mbed.h"

// Constructeur:
Servo::Servo(PinName pin) : m_pin(pin) {}

// Active le servomoteur:
void Servo::enable(int startPos, float period) {
    m_position = startPos;
    m_pulse.attach_us(callback(this, &Servo::startPulse), period);
}

// Désactive le servomoteur:
void Servo::disable() {
    m_pulse.detach();
}

// Déplace le servomoteur à la position angle (en °):
void Servo::pos(int angle) {
    // Conversion degrée en durée d'impulsion:
    m_position = -11.111f * angle + 2500;
}

// Début d'une impulsion:
void Servo::startPulse() {
    m_pin = 1;
    m_pulseStop.attach_us(callback(this, &Servo::endPulse), m_position);
}

// Fin de l'impulsion:
void Servo::endPulse() {
    m_pin = 0;
}
