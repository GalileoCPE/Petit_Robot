#include "Base.h"

// Constructeur:
Base::Base(Moteur* moteurG, Moteur* moteurD, Odometrie* odometrie, int update_delay_us): 
    m_asservissement_distance(1, 1, 1), m_asservissement_orientation(1, 1, 1) { 
    
    m_moteurG = moteurG;
    m_moteurD = moteurD;
    m_moteurs_isEnabled = false;

    // Odométrie:
    m_odometrie = odometrie;
    
    // Asservissement:
    m_pos_distance = 0;
    m_pos_orientation = 0;
    m_moteurG->setPWM_max(0.4);
    m_moteurD->setPWM_max(0.4);
    m_moteurG->setOffset(0.1);
    m_moteurD->setOffset(0.1);
    m_pwm_dynamic = 0.2f;  // max = 0.6 - 0.1
    
    // Erreur maximale:
    m_max_erreur_distance = 300;
    m_max_erreur_orientation = 3;
  
    // Répartition PWM entre distance et orientation:
    m_distance_orientation_ratio = 0.5f; // 50% -> distance, 50% -> orientation 

    // Mesure du déplacement toutes les <update_delay_us> µs:
    m_ticker.attach_us(callback(this, &Base::update), update_delay_us); // 2 kHz pour 500 us
}

// Setters:
void Base::setDistanceTo(float d) { m_asservissement_distance.toValue(d); }
void Base::setOrientationTo(float o) { m_asservissement_orientation.toValue(M_PI * o / 180.0f ); }

void  Base::setPID_distance(float P, float I, float D) {
    m_asservissement_distance.setPID(P, I, D);
}
void  Base::setPID_orientation(float P, float I, float D) {
    m_asservissement_orientation.setPID(P, I, D);
}

// Getters:
float* Base::getErreur_distance_ptr() { return &m_erreur_distance; }
float* Base::getErreur_orientation_ptr() { return &m_erreur_orientation; }
float* Base::getPID_PWM_distance_ptr() { return &m_pwm_distance; }
float* Base::getPID_PWM_orientation_ptr() { return &m_pwm_orientation; }
float* Base::getPID_PWM_gauche_ptr() { return &m_pwm_gauche; }
float* Base::getPID_PWM_droite_ptr() { return &m_pwm_droite; }

/*** Déplacement ***/

// Start:
void Base::start() {
    m_moteurs_isEnabled = true;
}

void Base::stop() {
  m_moteurs_isEnabled = false;
  
  // Arrêt des moteurs:
  m_moteurG->stop();
  m_moteurD->stop();
}


// Stop:

// Avant: (en mm)
void Base::forward(float d) {
    m_pos_distance += d;
    setDistanceTo(m_pos_distance);
}

// Arrière: (en mm)
void Base::backward(float d) {
    m_pos_distance -= d;
    setDistanceTo(m_pos_distance);
}

// Tourner: (en °)
void Base::turn(float o) {
    m_pos_orientation += o;
    setOrientationTo(m_pos_orientation);
}

// Renvoie le signe de la valeur:
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}


// Mise à jour de l'odometrie et de l'asservissement:
void Base::update() {
    // Mise à jour de la position:
    m_odometrie->update();
    
    // Calcul de l'erreur de distance:
    float distance = m_odometrie->getRealDistance();
    float erreur_distance = m_asservissement_distance.computeError(distance);
    
    // Calcul de l'erreur d'orientation:
    float theta = m_odometrie->getTheta();
    float erreur_theta = m_asservissement_orientation.computeError(theta);
    
    // Correction de l'erreur:
    if(abs(erreur_distance) > m_max_erreur_distance) erreur_distance = m_max_erreur_distance * sign(erreur_distance);    
    if(abs(erreur_theta) > m_max_erreur_orientation) erreur_theta = m_max_erreur_orientation * sign(erreur_theta);
    
    // PWM correspondant à la distance et à l'orientation:
    float pwm_distance = (erreur_distance / m_max_erreur_distance) * m_distance_orientation_ratio;
    float pwm_theta = (erreur_theta / m_max_erreur_orientation) * (1.0f - m_distance_orientation_ratio);
    
    // PWM à envoyer aux moteurs:
    float pwm_droite = (pwm_distance + pwm_theta) * m_pwm_dynamic;
    float pwm_gauche = (pwm_distance - pwm_theta) * m_pwm_dynamic;
    
    // Commande des moteurs:
    if (m_moteurs_isEnabled == true) {       
        m_moteurD->turn(pwm_droite);
        m_moteurG->turn(pwm_gauche);
    }
    
    // Sauvegarde / Debug:
    m_pwm_distance = pwm_distance;
    m_pwm_orientation = pwm_theta;
    m_erreur_distance = erreur_distance;
    m_erreur_orientation = erreur_theta;
    m_pwm_droite = pwm_droite;
    m_pwm_gauche = pwm_gauche;
}

// Getters:
float Base::getErreurDistance() { return m_erreur_distance; }
float Base::getErreurOrientation() { return m_erreur_orientation; }

//float Base::moveTo(float X, float Y);
//float Base::setAngle();

//float Base::turn(float angle);