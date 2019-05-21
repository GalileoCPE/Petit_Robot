#include "Moteur.h"

Moteur::Moteur(PinName pin_en, PinName pin_in1, PinName pin_in2):
  m_pwm(pin_en), m_in1(pin_in1, 0), m_in2(pin_in2, 0) {

    m_pwm.period_us(100.0f);  // Frequency = 10 kHz
    
    // PWM maximale qui peut être envoyée avec la commande turn():
    m_pwm_max = 0.5;
    
    m_offset = 0.0f;
}

// Setter:
void Moteur::setPWM(float pwm) { m_pwm = pwm + m_offset; }
void Moteur::setPWM_max(float pwm) { m_pwm_max = pwm + m_offset; }
void Moteur::setOffset(float offset) { m_offset = offset; }

/*** Déplacement ***/

// Avant:
void Moteur::forward() {
    m_in1 = 1;
    m_in2 = 0;
}

// Arrière:
void Moteur::backward() {
    m_in1 = 0;
    m_in2 = 1;
}

// Opposé:
void Moteur::opposite() {
    m_in1 = !m_in1.read();
    m_in2 = !m_in2.read();
}

// Arrêt:
void Moteur::stop() {
    m_pwm.write(0.0f);  // Duty cycle
}

// Avance ou recule en fonction du pwm:
void Moteur::turn(float pwm) {
    if (pwm < 0) {
        pwm -= m_offset;
        if (pwm < -m_pwm_max) pwm = -m_pwm_max;
        setPWM(-pwm);
        backward();
    } else {
        pwm += m_offset;
        if (pwm > m_pwm_max) pwm = m_pwm_max;
        setPWM(pwm);
        forward();
    }
}
