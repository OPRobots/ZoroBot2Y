#include "debug.h"
#include "sensores.h"

#define MODOS_DEBUG 3
int modo_debug = 0;

void debug_inicio() {
  if (boton_D()) {
    modo_debug = (modo_debug + 1) % MODOS_DEBUG;
    while (boton_D()) {
    }
  }
  if (modo_debug == 0) {
    for (int i = 0; i < 20; i++) {
      filtro_sensores();
      delayMicroseconds(1000 / 20);
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
  if (modo_debug == 1) {
      digitalWrite(LED_DERECHA, true);
      digitalWrite(LED_ADELANTE, false);
      digitalWrite(LED_IZQUIERDA, true);
    imprimir_sensores_raw();
  }
  if (modo_debug == 2) {
      digitalWrite(LED_DERECHA, true);
      digitalWrite(LED_ADELANTE, true);
      digitalWrite(LED_IZQUIERDA, true);
    imprimir_sensores_filtrados_analog();
  }
}

void imprimir_sensores_raw() {
  Serial.print(analogRead(S_PARED_1));
  Serial.print(" ");
  Serial.print(analogRead(S_PARED_2));
  Serial.print(" ");
  Serial.println(analogRead(S_PARED_3));

  delay(85);
}

void imprimir_sensores_filtrados() {
  for (int i = 0; i < 20; i++) {
    filtro_sensores();
    delayMicroseconds(1000 / 20);
  }
  Serial.print(sensor1());
  Serial.print(" ");
  Serial.print(sensor2());
  Serial.print(" ");
  Serial.println(sensor3());

   delay(85);
}
void imprimir_sensores_filtrados_analog() {
  for (int i = 0; i < 20; i++) {
    filtro_sensores();
    delayMicroseconds(1000 / 20);
  }
  Serial.print(sensor1_analog());
  Serial.print(" ");
  Serial.print(sensor2_analog());
  Serial.print(" ");
  Serial.println(sensor3_analog());

   delay(10);
}
