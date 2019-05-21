#include "Asservissement.h"

// Constructeur:
Asservissement::Asservissement(float P, float I, float D) {
  // PID:
  m_P = P;
  m_I = I;
  m_D = D;
  
  m_erreur_integral = 0;
  
  printf("Asservissement:\r\n");
  printf("  - P: %p\r\n", &m_P);
  printf("  - I: %p\r\n", &m_I);
  printf("  - D: %p\r\n", &m_D);
}

// Setters:
void Asservissement::setP(float P) { m_P = P; }
void Asservissement::setI(float I) { m_I = I; }
void Asservissement::setD(float D) { m_D = D; }
void Asservissement::setPID(float P, float I, float D) {
  m_P = P;
  m_I = I;
  m_D = D;
}

// Pointeurs:
float* Asservissement::getP_ptr() { return &m_P; }
float* Asservissement::getI_ptr() { return &m_I; }
float* Asservissement::getD_ptr() { return &m_D; }

// Asservissement:
void Asservissement::toValue(float value) {
  m_desired_value = value;
  m_erreur_integral = 0;
}

// Calcul de l'erreur:
float Asservissement::computeError(float value) {
  // Erreur de valeur:
  float erreur = m_desired_value - value;

  // Dérivée:
  float delta_erreur = m_erreur_prev - erreur;

  // Intégrale:
  m_erreur_integral += erreur;

  // Erreur totale:
  float total_erreur = m_P * erreur + m_I * m_erreur_integral + m_D * delta_erreur;

  // Enregistrement:
  m_erreur_prev = erreur;

  return total_erreur;
}
