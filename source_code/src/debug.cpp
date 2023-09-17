#include "debug.h"
#include "sensores.h"

#define MODOS_DEBUG 3
int modo_debug = 0;

void debug_inicio(){
  if (boton_D()) {
      modo_debug = (modo_debug + 1) % MODOS_DEBUG;
      while (boton_D()) {
      }
    }
    if (modo_debug == 0) {
      for (int i = 0; i < 20; i++) {
        filtro_sensores();
      }
      if (sensor1()) {
        digitalWrite(LED_IZQUIERDA, true);
      } else {
        digitalWrite(LED_IZQUIERDA, false);
      }
      if (sensor2()) {
        digitalWrite(LED_ADELANTE, true);
      } else {
        digitalWrite(LED_ADELANTE, false);
      }
      if (sensor3()) {
        digitalWrite(LED_DERECHA, true);
      } else {
        digitalWrite(LED_DERECHA, false);
      }
    }
    if (modo_debug == 2) {
      imprimir_sensores_raw();
    }
}

void imprimir_sensores_raw() {
  Serial.print(analogRead(S_PARED_1));
  Serial.print(" ");
  Serial.print(analogRead(S_PARED_2));
  Serial.print(" ");
  Serial.print(analogRead(S_PARED_3));
  Serial.print(" ");

  delay(125);
}

void imprimir_sensores_filtrados() {
  Serial.print(sensor1());
  Serial.print(" ");
  Serial.print(sensor2());
  Serial.print(" ");
  Serial.print(sensor3());
  Serial.print(" ");

  //delay(200);
}
void imprimir_sensores_filtrados_analog() {
  Serial.print(sensor1_analog());
  Serial.print(" ");
  Serial.print(sensor2_analog());
  Serial.print(" ");
  Serial.print(sensor3_analog());
  Serial.print(" ");

  //delay(200);
}

