#include "mbed.h"

// Eléments:
#include <math.h>
#include "Base.h"
#include "Encodeur.h"
#include "Infra.h"
#include "Moteur.h"
#include "Odometrie.h"
#include "Servo.h"

/*** Paramètres ***/

// Pin tirette:
#define pin_tirette A4

// Pin interrupteur:
#define pin_switch_color A5

// Pin Servo:
#define pin_servo A0

// Pins des moteurs:
#define pins_moteurG_enA_in1_in2 D8, D11, D12
#define pins_moteurD_enB_in1_in2 D7, D14, D15

// Pin pompe à air:
#define pin_pompe D6

// Pins Infrarouge:
#define pin_infra1 A1
#define pin_infra2 A2
#define pin_infra3 A3

// Paramètres des encodeurs:
#define encodeur_res 1024
#define encodeur_diametre 40         // en mm
#define encodeurs_entraxe 97 / 2.0f  // en mm

// Paramètre de l'odometrie et asservissement:
#define update_delay_us 500  // en µs

/*** Création des objets ***/

// Moteurs:
Moteur moteurG(pins_moteurG_enA_in1_in2);
Moteur moteurD(pins_moteurD_enB_in1_in2);

// Encodeurs:
Encodeur encodeurG('G', encodeur_res, encodeur_diametre);
Encodeur encodeurD('D', encodeur_res, encodeur_diametre);

// Odometrie:
Odometrie odometrie(&encodeurG, &encodeurD, encodeurs_entraxe);

// Base:
Base base(&moteurG, &moteurD, &odometrie, update_delay_us);
#define base_max_erreur_distance 10
#define base_max_erreur_orientation 0.1f

// Tirette:
DigitalIn tirette(pin_tirette);

// Interrupteur:
DigitalIn switch_color(pin_switch_color);

// Pompe:
DigitalOut pompe(pin_pompe);

// Servo:
Servo servo(pin_servo);

// Infrarouge:
Infra infraFrontD(pin_infra1);
Infra infraFrontG(pin_infra2);
Infra infraBack(pin_infra3);
int detection();
bool isStopped = false;

// Fin de l'épreuve:
#define epreuve_duration 100  // s
Timeout epreuve_timeout;
void finish() { base.stop(); }

// Prototypes - Machines d'états:
extern int PURPLE_steps(int state);
extern int YELLOW_steps(int state);

int main() {
  printf("STM32 Petit robot\r\n");
  printf("Adresses de variables: (pour STM-Studio)\r\n");
  printf(" - encodeurG.tours   = %p\r\n", encodeurG.getTours_ptr());
  printf(" - encodeurD.tours   = %p\r\n", encodeurD.getTours_ptr());
  printf(" - odometrie.X       = %p\r\n", odometrie.getX_ptr());
  printf(" - odometrie.Y       = %p\r\n", odometrie.getY_ptr());
  printf(" - odometrie.Theta   = %p\r\n", odometrie.getTheta_ptr());
  printf(" - base.erreur_distance    = %p\r\n", base.getErreur_distance_ptr());
  printf(" - base.erreur_orientation = %p\r\n", base.getErreur_orientation_ptr());
  printf(" - base.pwm_distance       = %p\r\n", base.getPID_PWM_distance_ptr());
  printf(" - base.pwm_orientation    = %p\r\n", base.getPID_PWM_orientation_ptr());
  printf(" - base.pwm_gauche   = %p\r\n", base.getPID_PWM_gauche_ptr());
  printf(" - base.pwm_droite   = %p\r\n", base.getPID_PWM_droite_ptr());

  // PID:
  base.setPID_distance(1, 0, 1);
  base.setPID_orientation(10, 0, 1);

  base.start();

  // Tests:
  base.forward(1000);
  while (1) wait(1);

  base.turn(360.0f*10);
  wait(6);
  base.turn(90.0f);
  wait(3);
  base.forward(-600);
  wait(3);

  while (1) wait(1);

  /*while(1) {
    infraFrontG.mesure();
    infraFrontD.mesure();
    infraBack.mesure();
    float distanceG = infraFrontG.getDistance();
    float distanceD = infraFrontD.getDistance();
    float distanceB = infraBack.getDistance();
    

    printf("f_G: %f \t f_D: %f \t f_B: %f\r\n", distanceG, distanceD,
  distanceB); wait(0.1);
  }*/

  // Démarre le servo:
  servo.enable(1500, 20000);
  servo.pos(90);

  // Tirette:
  char count = 0;
  while (count < 5) {
    if(!tirette) count++;
    else count = 0;
    wait(0.1);
  }

  wait(1);

  // Fin de l'épreuve:
  epreuve_timeout.attach(&finish, epreuve_duration);

  // Couleur du robot:
  bool color_side = 1;  // switch_color;

  // Démarre les moteurs:
  base.start();

  // Eteint la pompe:
  pompe = 0;

  // Désactive capteur arrière:
  infraBack.off();

  // Déplacement:
  int time = 0, timeOut = 500, state = 0;
  while (1) {
    // Gestion du temps:
    wait_ms(1);

    // Détection:
    if (time % 50 == 0 || isStopped)
      time += detection();
    else
      time++;

    // Etape suivante:

    // Récupère les erreurs:
    // float erreur_distance = base.getErreurDistance();
    // float erreur_orientation = base.getErreurOrientation();
    // if (time % 100 == 0) printf("d: %f, o: %f\r\n", erreur_distance,
    // erreur_orientation);

    // Si la position est respectée:
    // if (state <= 11 && (time > timeOut || abs(erreur_distance) <
    // base_max_erreur_distance && abs(erreur_orientation) <
    // base_max_erreur_orientation)){
    if (state <= 14 && time > timeOut) {
      if (color_side == 0)
        timeOut = PURPLE_steps(state);
      else
        timeOut = YELLOW_steps(state);

      state++;
      time = 0;
    }
  }
}

// Détection de l'environnement;
int detection() {
  bool detectG = infraFrontG.mesure();
  bool detectD = infraFrontD.mesure();
  bool detectB = infraBack.mesure();

  // if (detectG || detectD || detectB) {
  if (detectG || detectD) {
    base.stop();
    buzzer_on();

    // Si première détection:
    if (isStopped == false) {
      isStopped = true;
      moteurG.setPWM(0.1);
      moteurD.setPWM(0.1);
      moteurG.forward();
      moteurD.forward();
      wait(0.5);
      moteurD.stop();
      moteurG.stop();
    } else {
      wait(1);
    }

  } else if (detectB) {
    base.stop();
    buzzer_on();

    // Si première détection:
    if (isStopped == false) {
      isStopped = true;
      moteurG.setPWM(0.1);
      moteurD.setPWM(0.1);
      moteurG.backward();
      moteurD.backward();
      wait(0.5);
      moteurD.stop();
      moteurG.stop();
    } else {
      wait(1);
    }
  } else {
    if (isStopped == true) base.start();
    buzzer_off();
    isStopped = false;
    return 1;
  }

  return 0;
}
