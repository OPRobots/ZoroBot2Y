#ifndef __PINES_H
#define __PINES_H
#include "Arduino.h"

// PINES
// Sensores numerados por orden de colocación en el robot mirandolo desde arriba con los sensores adelante

#define S_PARED_1 0
#define S_PARED_2 1
#define S_PARED_3 2

#define PWM_MD_A 3
#define PWM_MD_B 4
#define PWM_MI_A 5
#define PWM_MI_B 6

#define LED_ADELANTE 7
#define LED_DERECHA 8
#define LED_IZQUIERDA 10

#define BOTON_D 20
#define BOTON_I 21


void inicializar_pines();

#endif