#include "Infra.h"
#include <math.h>

// Constructeur:
Infra::Infra(PinName pin)
    : m_analog(pin),
      m_isOn(true),
      m_distdetect(30.0f),
      m_detect(false) {}

// Mesure:
bool Infra::mesure() {
  // Mesure:
  m_voltage = m_analog.read();
  m_voltage = (float)m_voltage * (float)3.3;
  m_distance = (float)29.988 * powf(m_voltage, -1.173) / 2.0f;
  
  // Détection:
  if (m_isOn) m_detect = m_distance < m_distdetect;
  else m_detect = false;
  
  return m_detect;
}

// Setters:
void Infra::setDistDetect(float detect) { m_distdetect = detect; }
void Infra::on() { m_isOn = true; }
void Infra::off() { m_isOn = false; }

// Getters:
float Infra::getDistance() { return m_distance; }
float Infra::getVoltage() { return m_voltage; }
float Infra::getDistdetect() { return m_distdetect; }
