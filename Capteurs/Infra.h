#ifndef INFRA_H
#define INFRA_H

#include "mbed.h"
#include <vector>

class Infra {
 public:
  // Constructeur:
  Infra(PinName);

  // Mesure:
  bool mesure();
  
  // Setters:
  void setDistDetect(float);
  void on();
  void off();

  // Getters:
  float getDistance();
  float getDistdetect();
  float getVoltage();

 private:
  AnalogIn m_analog;
  bool m_isOn;
  float m_voltage;
  float m_distance;
  float m_distdetect;
  bool m_detect;
};
#endif