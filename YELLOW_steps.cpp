// Eléments:
#include "Base.h"
#include "Infra.h"
#include "Servo.h"

extern Base base;
extern Servo servo;
extern Infra infraFrontG;
extern Infra infraFrontD;
extern Infra infraBack;
extern DigitalOut pompe;

// Déplacement pour côté Jaune:
int YELLOW_steps(int state) {
  int timeOut = 3000;

  // 0: Avant:
  if (state == 0) {
    base.forward(-1700);
    timeOut = 6000;
  }

  // 1: Tourne:
  else if (state == 1) {
    base.turn(90.0f);
    infraFrontG.off();
    infraFrontD.off();
    infraBack.on();
  }

  // 2: Avance vers palet:
  else if (state == 2) {
    servo.pos(10);
    base.forward(-470);
    timeOut = 3000;
  }

  // 3: Servo:
  else if (state == 3) {
    // base.stop();
    servo.pos(45);
    timeOut = 3000;
  }

  // 4: Recule du palet:
  else if (state == 4) {
    base.start();
    base.backward(-200);
    timeOut = 3000;
  }

  // 5: Tourne
  else if (state == 5) {
    base.start();
    base.turn(90.0f);
  }

  // 6: Avance
  else if (state == 6) {
    base.backward(-520);
    infraFrontG.on();
    infraFrontD.on();
  }

  // 7: Tourne
  else if (state == 7) {
    base.turn(-90.0f);
    servo.pos(130.0f);
    infraFrontG.off();
    infraFrontD.off();
  }

  // 8: Avance
  else if (state == 8)
    base.forward(-400.0f);

  // 9: Pompe à air
  else if (state == 9) {
    pompe = 1;
    timeOut = 2000;
  }

  // 10: Recule du Goldenium
  else if (state == 10) {
    base.backward(-500.0f);
    timeOut = 1000;
  }

  // 11: Désactive la pompe
  else if (state == 11) {
    pompe = 0;
    timeOut = 1000;
  }

  // 12: Tourne pour revenir
  else if (state == 12) {
    base.turn(90.0f);
  }

  // 13: Avance pour revenir
  else if (state == 13) {
    base.forward(-1700);
    infraFrontG.on();
    infraFrontD.on();
    timeOut = 6000;
  }

  // 14: Fin
  else if (state == 14) {
    base.stop();
  }

  return timeOut;
}