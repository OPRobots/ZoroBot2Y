#ifndef __SENSORES_H
#define __SENSORES_H
#include <Arduino.h>
#include "pines.h"

void no_filtro_sensores();
void filtro_sensores();
bool sensor1();
bool sensor2();
bool sensor3();
int sensor1_analog();
int sensor2_analog();
int sensor3_analog();
bool boton_D();
bool boton_I();

#endif