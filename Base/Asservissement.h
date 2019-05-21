#ifndef ASSERVISSEMENT_H
#define ASSERVISSEMENT_H

#include "mbed.h"

class Asservissement {
 public:
  // Constructeur:
  Asservissement(float P, float I, float D);
  
  // Setters:
  void setP(float P);
  void setI(float I);
  void setD(float D);
  void setPID(float P, float I, float D);
  
  // Pointeurs:
  float* getP_ptr();
  float* getI_ptr();
  float* getD_ptr();
  
  // Asservissement:
  void toValue(float value);
  
  // Calcul d'asservissement:
  float computeError(float value);

 private:  
  // PID:
  float m_P, m_I, m_D;
  
  // Value:
  float m_desired_value;
  
  float m_erreur_prev, m_erreur_integral;
  
};

#endif