#include "sensores.h"
#include "pines.h"

const int MAGNITUD_FILTRO = 20;
const int UMBRAL_FILTRO = 2400;
const int UMBRAL = 2750;
const int CONTADOR = 3;


int contador_frontal = 0;
int contador_boton_D = 0;
int contador_boton_I = 0;

bool s1_bool = false;
bool s2_bool = false;
bool s3_bool = false;

int s1 = 0;
int s2 = 0;
int s3 = 0;

int s1_aux = 0;
int s2_aux = 0;
int s3_aux = 0;

int Filtro_s1[MAGNITUD_FILTRO];
int Filtro_s2[MAGNITUD_FILTRO];
int Filtro_s3[MAGNITUD_FILTRO];

int i_s = 0;

void no_filtro_sensores() {

  s1_aux = analogRead(S_PARED_1);
  s2_aux = analogRead(S_PARED_2);
  s3_aux = analogRead(S_PARED_3);

  if (s1_aux < UMBRAL_FILTRO) {
    s1_aux = UMBRAL_FILTRO;
  }
  if (s2_aux < UMBRAL_FILTRO) {
    s2_aux = UMBRAL_FILTRO;
  }
  if (s3_aux < UMBRAL_FILTRO) {
    s3_aux = UMBRAL_FILTRO;
  }

  s1 = map(s1_aux, UMBRAL_FILTRO, 3000, 0, 1000);
  s2 = map(s2_aux, UMBRAL_FILTRO, 3000, 0, 1000);
  s3 = map(s3_aux, UMBRAL_FILTRO, 3000, 0, 1000);
}

void filtro_sensores() {

  s1_aux = analogRead(S_PARED_1);
  s2_aux = analogRead(S_PARED_2);
  s3_aux = analogRead(S_PARED_3);

  if (s1_aux > UMBRAL_FILTRO) {
    Filtro_s1[i_s] = s1_aux;
  } else {
    Filtro_s1[i_s] = UMBRAL_FILTRO;
  }
  if (s2_aux > UMBRAL_FILTRO) {
    Filtro_s2[i_s] = s2_aux;
  } else {
    Filtro_s2[i_s] = UMBRAL_FILTRO;
  }
  if (s3_aux > UMBRAL_FILTRO) {
    Filtro_s3[i_s] = s3_aux;
  } else {
    Filtro_s3[i_s] = UMBRAL_FILTRO;
  }

  i_s = (i_s + 1) % MAGNITUD_FILTRO; // Avanza el índice circularmente cuando supera MAGNITUD FILTRO vuelve a ser 0

  s1 = 0;
  s2 = 0;
  s3 = 0;

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

  s1 = map(s1, UMBRAL_FILTRO, 3000, 0, 1000);
  s2 = map(s2, UMBRAL_FILTRO, 3000, 0, 1000);
  s3 = map(s3, UMBRAL_FILTRO, 3000, 0, 1000);
}

bool sensor1() {
  return s1_bool;
}

bool sensor2() {
if (s2_bool) {
    contador_frontal++;
  } else {
    contador_frontal = 0;
  }
  if (contador_frontal >= CONTADOR) {
    return true;
  } else {
    return false;
  }
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
    delay(5);
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
    delay(5);
  } else {
    contador_boton_I = 0;
  }
  if (contador_boton_I >= CONTADOR) {
    return true;
  } else {
    return false;
  }
}