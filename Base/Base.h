#ifndef BASE_H
#define BASE_H

#include "mbed.h"
#include "Moteur.h"
#include "Odometrie.h"
#include "Asservissement.h"

#ifndef M_PI
#define M_PI   3.14159265358979323846f
#endif

class Base {
 public:
  // Constructeur:
  Base(Moteur*, Moteur*, Odometrie*, int);
  
  // Déplacement:
  //float moveTo(float X, float Y);
  //float setAngle();
  //float forward(float d);
  void start();
  void stop();
  void forward(float);
  void backward(float);
  void turn(float); // °
  
  //float turn(float angle);
  
  // PID:
  void setPID_distance(float P, float I, float D);
  void setPID_orientation(float P, float I, float D);
  
  // Pointeurs:
  float* getErreur_distance_ptr();
  float* getErreur_orientation_ptr();
  float* getPID_PWM_distance_ptr();
  float* getPID_PWM_orientation_ptr();
  float* getPID_PWM_gauche_ptr();
  float* getPID_PWM_droite_ptr();
  
  // Getters:
  float getErreurDistance();
  float getErreurOrientation();

  
 private:
  // Moteurs:
  Moteur* m_moteurG;
  Moteur* m_moteurD;
  bool m_moteurs_isEnabled;
  
  // Odometrie:
  Odometrie* m_odometrie;
 
  // Asservissement distance et orientation:
  Asservissement m_asservissement_distance;
  Asservissement m_asservissement_orientation;
  float m_pos_distance;
  float m_pos_orientation;
  void setDistanceTo(float);
  void setOrientationTo(float o);
  //Asservissement m_asservissement_vitesse;
  float m_max_erreur_distance, m_max_erreur_orientation; 
  float m_distance_orientation_ratio;
  
  // PWM maximale envoyée au moteurs:
  float m_pwm_max;
  float m_pwm_dynamic;
  
  // Debug:
  float m_erreur_distance, m_erreur_orientation;
  float m_pwm_gauche, m_pwm_droite;
  float m_pwm_distance, m_pwm_orientation;
  
  // Mise à jour de la position:
  void update();

  // Ticker:
  Ticker m_ticker;
};

#endif