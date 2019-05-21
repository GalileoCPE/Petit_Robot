#include "Encodeur.h"

// Timers & Encoders:
TIM_HandleTypeDef timer3, timer4;
TIM_Encoder_InitTypeDef encoder3, encoder4;

// Constructeur:
Encodeur::Encodeur(char e, int resolution, int diametre) {
  TIM_Encoder_InitTypeDef* encodeur;
  TIM_TypeDef* TIMx;
  
  if (e == 'G') {
    encodeur = &encoder3;
    TIMx = TIM3;
    m_timer = &timer3;
  } else if (e == 'D') {
    encodeur = &encoder4;
    TIMx = TIM4;
    m_timer = &timer4;
  }
  m_resolution = resolution * 4 - 1; // *4 car l'encodeur détecte tous les fronts des 2 signaux
  m_diametre = diametre;
  m_ticks_par_mm = (float)m_resolution / (float)(M_PI * m_diametre);
  
  m_tours = -1;  // -1 -> 0 au démarrage (car une interruption est déclenchée)
  
  EncodeurInit(encodeur, m_timer, TIMx, m_resolution);
}

// Getters & Setters:
uint16_t Encodeur::getCount() { return __HAL_TIM_GET_COUNTER(m_timer); }
int Encodeur::getTours() { return m_tours; }
int* Encodeur::getTours_ptr() { return &m_tours; }
void Encodeur::updateTour(int dtour) { m_tours += dtour; }

int Encodeur::getDist() { return M_PI * m_diametre * (m_tours + (float)getCount() / (float)m_resolution); }
int Encodeur::getTotalCount() {
    int encodeur_tour = m_tours;
    
    // Détecte si le compteur s'est réinitialisé sans ajouter le tour :
    // (c'est-à-dire TIMx_IRQHandler ne s'est pas encore déclenché)
    if (__HAL_TIM_GET_FLAG(m_timer, TIM_FLAG_UPDATE) != RESET) {
        // Récupère la direction de rotation de l'encodeur:
        bool direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(m_timer);

        // Ajoute ou enlève un tour:
        encodeur_tour += direction ? -1 : 1;
    }

    return encodeur_tour * m_resolution + getCount();
}

int Encodeur::getDiametre() { return m_diametre; }
int Encodeur::getResolution() { return m_resolution; }
float Encodeur::getTicks_par_mm() { return m_ticks_par_mm; }

// Initialisation du Timer pour l'Encodeur:
void EncodeurInit(TIM_Encoder_InitTypeDef* encodeur, TIM_HandleTypeDef* timer, TIM_TypeDef* TIMx, uint32_t resolution) {
    // Timer:
    timer->Instance = TIMx;
    timer->Init.Prescaler = 0;
    timer->Init.Period = resolution;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    
    // Encodeur:
    encodeur->EncoderMode = TIM_ENCODERMODE_TI12;
    
    // Channel 1:
    encodeur->IC1Filter = 0;
    encodeur->IC1Polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    encodeur->IC1Prescaler = TIM_ICPSC_DIV1;
    encodeur->IC1Selection = TIM_ICSELECTION_DIRECTTI;
    
    // Channel 2:
    encodeur->IC2Filter = 0;
    encodeur->IC2Polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    encodeur->IC2Prescaler = TIM_ICPSC_DIV1;
    encodeur->IC2Selection = TIM_ICSELECTION_DIRECTTI;

    if (HAL_TIM_Encoder_Init(timer, encodeur) != HAL_OK) {
        printf("Couldn't Init Encoder\r\n");
        while (1) {}
    }

    if (HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_1)!= HAL_OK) {
        printf("Couldn't Start Encoder\r\n");
        while (1) {}
    }
    
    // Active l'interruption du timer:
    HAL_TIM_Base_Start_IT(timer);
}


// Fonction d'interruptions pour compter le nombre de tours:
extern Encodeur encodeurG, encodeurD;
extern "C"
{
    // Gestion de l'interruption du timer 3:
    void TIM3_IRQHandler(void) {
        HAL_TIM_IRQHandler(&timer3);

        // Récupère la direction de rotation de l'encodeur:
        bool direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer3);

        // Ajoute ou enlève un tour:
        encodeurG.updateTour(direction ? -1 : 1);
    }
    
    // Gestion de l'interruption du timer 4:
    void TIM4_IRQHandler(void) {
        HAL_TIM_IRQHandler(&timer4);

        // Récupère la direction de rotation de l'encodeur:
        bool direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer4);

        // Ajoute ou enlève un tour:
        encodeurD.updateTour(direction ? -1 : 1);
    }
}