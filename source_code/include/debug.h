#ifndef __DEBUG_H
#define __DEBUG_H
#include <Arduino.h>
#include "pines.h"
#include "sensores.h"

void debug_inicio();
void imprimir_sensores_raw();
void imprimir_sensores_filtrados();
void imprimir_sensores_filtrados_analog();

#endif