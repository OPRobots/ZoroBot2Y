#include "sensores.h"
#include "pines.h"

const int MAGNITUD_FILTRO = 20;
const int UMBRAL = 1700;
const int CONTADOR = 6;

int contador_boton_D = 0;
int contador_boton_I = 0;

bool s1_bool = false;
bool s2_bool = false;
bool s3_bool = false;

int s1 = 0;
int s2 = 0;
int s3 = 0;

int Filtro_s1[MAGNITUD_FILTRO];
int Filtro_s2[MAGNITUD_FILTRO];
int Filtro_s3[MAGNITUD_FILTRO];

int i_s = 0;

void filtro_sensores() {

  Filtro_s1[i_s] = analogRead(S_PARED_1);
  Filtro_s2[i_s] = analogRead(S_PARED_2);
  Filtro_s3[i_s] = analogRead(S_PARED_3);
  i_s = (i_s + 1) % MAGNITUD_FILTRO; // Avanza el índice circularmente cuando supera MAGNITUD FILTRO vuelve a ser 0

  for (int i = 0; i < MAGNITUD_FILTRO; i++) {
    s1 += Filtro_s1[i];
    s2 += Filtro_s2[i];
    s3 += Filtro_s3[i];
  }

  s1 = s1 / MAGNITUD_FILTRO;
  s2 = s2 / MAGNITUD_FILTRO;
  s3 = s3 / MAGNITUD_FILTRO;

  s1_bool = s1 > UMBRAL;
  s2_bool = s2 > UMBRAL;
  s3_bool = s3 > UMBRAL;
}

bool sensor1() {
  return s1_bool;
}
bool sensor2() {
  return s2_bool;
}
bool sensor3() {
  return s3_bool;
}

int sensor1_analog() {
  return s1;
}
int sensor2_analog() {
  return s2;
}
int sensor3_analog() {
  return s3;
}

bool boton_D() {
  if (!digitalRead(BOTON_D)) {
    contador_boton_D++;
  } else {
    contador_boton_D = 0;
  }
  if (contador_boton_D >= CONTADOR) {
    return true;
  } else {
    return false;
  }
}

bool boton_I() {
  if (!digitalRead(BOTON_I)) {
    contador_boton_I++;
  } else {
    contador_boton_I = 0;
  }
  if (contador_boton_I >= CONTADOR) {
    return true;
  } else {
    return false;
  }
}