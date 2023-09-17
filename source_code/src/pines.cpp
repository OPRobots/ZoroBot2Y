#include "pines.h"

// PINES
// Sensores numerados por orden de colocación en el robot mirandolo desde arriba con la cuchilla hacia adelante

void inicializar_pines(){
  pinMode(S_PARED_1, INPUT);
  pinMode(S_PARED_2, INPUT);
  pinMode(S_PARED_3, INPUT);

  pinMode(PWM_MD_A, OUTPUT);
  pinMode(PWM_MD_B, OUTPUT);
  pinMode(PWM_MI_A, OUTPUT);
  pinMode(PWM_MI_B, OUTPUT);

  pinMode(LED_ADELANTE, OUTPUT);
  pinMode(LED_DERECHA, OUTPUT);
  pinMode(LED_IZQUIERDA, OUTPUT);

  pinMode(BOTON_D, INPUT_PULLUP);
  pinMode(BOTON_I, INPUT_PULLUP);
}
