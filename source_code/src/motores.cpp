#include "motores.h"

int MD_A = 0;
int MD_B = 1;
int MI_A = 2;
int MI_B = 3;

int Vel_D = 0;
int Vel_I = 0;

void inicializar_motores() {
  ledcSetup(MD_A, 20000, 10); // Canal de PWM (0 - 16),Frecuencia, Resolucion (nº bits)
  ledcSetup(MD_B, 20000, 10); // Canal de PWM (0 - 16),Frecuencia, Resolucion (nº bits)
  ledcSetup(MI_A, 20000, 10); // Canal de PWM (0 - 16),Frecuencia, Resolucion (nº bits)
  ledcSetup(MI_B, 20000, 10); // Canal de PWM (0 - 16),Frecuencia, Resolucion (nº bits)

  ledcAttachPin(PWM_MD_A, MD_A);
  ledcAttachPin(PWM_MD_B, MD_B);
  ledcAttachPin(PWM_MI_A, MI_A);
  ledcAttachPin(PWM_MI_B, MI_B);

  ledcWrite(MD_A, 0);
  ledcWrite(MD_B, 0);
  ledcWrite(MI_A, 0);
  ledcWrite(MI_B, 0);
}

void asignacion_vel_motores(int vel, int correccion) {
  int velI = vel - correccion;
  int velD = vel + correccion;

  velD = map(velD, -1000, 1000, -1024, 1024);
  velI = map(velI, -1000, 1000, -1024, 1024);

  // de tal modo que si la correccion hace que una rueda se ponga a mas de 255, se limita a 255 y a la otra se le aplique la correccion restante
  if (velD > 1024) {
    velI = velI - (velD - 1024);
    velD = 1024;
  }
  if (velI > 1024) {
    velD = velD - (velI - 1024);
    velI = 1024;
  }

  // asiganmos valores a la rueda derecha teniendo en cuenta de que si el valor es negativo va hacia atras
  if (velD >= 0) {
    ledcWrite(MD_A, 1024);
    ledcWrite(MD_B, 1024 - velD);
    // Serial.print(255 - velD);
  } else {
    ledcWrite(MD_A, 1024 - abs(velD));
    ledcWrite(MD_B, 1024);
    // Serial.println(255 - abs(velD));
  }

  // Serial.print(" ");
  // asiganmos valores a la rueda izquierda teniendo en cuenta de que si el valor es negativo va hacia atras
  if (velI >= 0) {
    ledcWrite(MI_A, 1024);
    ledcWrite(MI_B, 1024 - velI);
    // Serial.print(255 - velD);
  } else {
    ledcWrite(MI_A, 1024 - abs(velI));
    ledcWrite(MI_B, 1024);
    // Serial.println(255 - abs(velD));
  }
}
