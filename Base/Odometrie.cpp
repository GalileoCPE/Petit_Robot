#include "Odometrie.h"

Odometrie::Odometrie(Encodeur* encodeurG, Encodeur* encodeurD, float entraxe) {
    m_encodeurG = encodeurG;
    m_encodeurD = encodeurD;
    
    m_L = 0.0f;
    m_X = 0.0f;
    m_Y = 0.0f;
    m_Theta = 0; //3.14159265359; // PI
    m_vitesse = 0.0f;
    m_distance = 0.0f;
    
    m_prev_encodeurG_count = m_encodeurG->getTotalCount();
    m_prev_encodeurD_count = m_encodeurD->getTotalCount();
    
    // Nombre d'impulsions par mm parcourus:
    m_ticks_par_mm = m_encodeurD->getTicks_par_mm();
    
    // Entraxe des 2 encodeurs en ticks:
    m_entraxe_ticks = entraxe * m_ticks_par_mm;
    
    printf("Ticks par mm : %f\n\r", m_ticks_par_mm);
    printf("Entraxe en ticks : %f\n\r", m_entraxe_ticks);
}

// Getters:
float Odometrie::getX() { return m_X; }
float Odometrie::getY() { return m_Y; }
float Odometrie::getTheta() { return m_Theta; }
float Odometrie::getDistance() { return m_distance; }
float Odometrie::getRealDistance() { return m_distance / m_ticks_par_mm; }
float Odometrie::getVitesse() { return m_vitesse; }

// Pointeurs:
float* Odometrie::getX_ptr() { return &m_X; }
float* Odometrie::getY_ptr() { return &m_Y; }
float* Odometrie::getTheta_ptr() { return &m_Theta; }
float* Odometrie::getDistance_ptr() { return &m_distance; }
float* Odometrie::getVitesse_ptr() { return &m_vitesse; }

// Selon le ClubElek:
void Odometrie::update() {
    
    // Récupère les compteurs des encodeurs:
    int encodeurG_count = m_encodeurG->getTotalCount();
    int encodeurD_count = m_encodeurD->getTotalCount();
    
    // Différences de ticks depuis le dernier update():
    int dEncodeurG = encodeurG_count - m_prev_encodeurG_count;
    int dEncodeurD = encodeurD_count - m_prev_encodeurD_count;
    
    // Différences de distance et d'angle:
    float dL = (dEncodeurD + dEncodeurG) / 2.0f;
    float dTheta = (dEncodeurD - dEncodeurG) / 2.0f;
    
    // Nouvel Angle:
    m_Theta += dTheta / m_entraxe_ticks;
    
    // Nouvelles Positions X et Y:
    float dX = dL * cos(m_Theta);
    float dY = dL * sin(m_Theta);
    m_X += dX / m_ticks_par_mm;
    m_Y += dY / m_ticks_par_mm; 
    
    // Vitesse:
    m_vitesse = dL;
    
    // Distance :
    m_distance += dL;
    
    // Sauvegarde:
    m_prev_encodeurG_count = encodeurG_count;
    m_prev_encodeurD_count = encodeurD_count;
}



