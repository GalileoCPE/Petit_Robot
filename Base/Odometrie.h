#ifndef ODOMETRIE_H
#define ODOMETRIE_H

#include "mbed.h"
#include "Encodeur.h"

class Odometrie {
 public:
  // Constructeur:
  Odometrie(Encodeur* encodeurG, Encodeur* encodeurD, float entraxe);
  
  void update();
  
  // Getters:
  float getX();
  float getY();
  float getTheta();
  float getDistance();
  float getRealDistance();
  float getVitesse();
  
  // Pointeurs:
  float* getX_ptr();
  float* getY_ptr();
  float* getTheta_ptr();
  float* getDistance_ptr();
  float* getVitesse_ptr();

 private:
  // Position et orientation:
  float  m_L, m_X, m_Y, m_Theta, m_distance, m_vitesse;
  
  // Encodeurs:
  Encodeur* m_encodeurG;
  Encodeur* m_encodeurD;
  float m_ticks_par_mm, m_entraxe_ticks;
  
  int m_prev_encodeurG_count, m_prev_encodeurD_count;
  
};

#endif