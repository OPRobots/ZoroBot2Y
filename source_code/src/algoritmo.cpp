#include "algoritmo.h"

#define REFERENCE_WALL_CHANGE_LENGTH 2 // numero de cambios

int millis_reference_wall_change_1 = 0;
int millis_reference_wall_change_2 = 0;
int millis_reference_wall_change_3 = 0;
int millis_reference_wall_change_4 = 0;
int millis_reference_wall_change_5 = 0;

int REFERENCE_WALL_CHANGE_MILLIS[] = {
    millis_reference_wall_change_1,
    millis_reference_wall_change_2,
    millis_reference_wall_change_3,
    millis_reference_wall_change_4,
    millis_reference_wall_change_5};

bool REFERENCE_WALL_CHANGE_DONE[] = {false, false, false, false, false};

int indice_cambios = 0;
bool tiempo_seg = true;
bool tiempo_mseg = false;
int patron_led = 0;
bool configuracion_lista = false;

bool check_reference_wall_change(long startedMillis, bool mano) {
  for (int i = 0; i < REFERENCE_WALL_CHANGE_LENGTH; i++) {
    if (!REFERENCE_WALL_CHANGE_DONE[i] && REFERENCE_WALL_CHANGE_MILLIS[i] != 0 && millis() > startedMillis + REFERENCE_WALL_CHANGE_MILLIS[i]) {
      mano = !mano;
      REFERENCE_WALL_CHANGE_DONE[i] = true;
      //Serial.print("cambio");
    }
  }
  if (mano) {
    digitalWrite(LED_IZQUIERDA, HIGH);
    digitalWrite(LED_DERECHA, LOW);
  } else {
    digitalWrite(LED_IZQUIERDA, LOW);
    digitalWrite(LED_DERECHA, HIGH);
  }
  return mano;
}

bool configuracion() {

  if (boton_I()) {
    if (tiempo_seg) {
      REFERENCE_WALL_CHANGE_MILLIS[indice_cambios] += 1000;
    } else if (tiempo_mseg) {
      REFERENCE_WALL_CHANGE_MILLIS[indice_cambios] += 100;
    }
    while (boton_I()) {
    }
    patron_led = (patron_led + 1) % 10;
    leds_configuracion();
  }
  if (boton_D()) {
    if (tiempo_seg) {
      tiempo_seg = false;
      tiempo_mseg = true;
      patron_led = 0;
      leds_configuracion();
    } else if (tiempo_mseg) {
      if (indice_cambios >= (REFERENCE_WALL_CHANGE_LENGTH - 1)) {
        configuracion_lista = true;
      } else {
        indice_cambios++;
        tiempo_seg = true;
        tiempo_mseg = false;
        REFERENCE_WALL_CHANGE_MILLIS[indice_cambios] = REFERENCE_WALL_CHANGE_MILLIS[indice_cambios - 1];

        patron_led = 0;
        leds_configuracion();
      }
    }
    while (boton_D()) {
    }
  }
  if (configuracion_lista) {
    for (int i = 0; i < REFERENCE_WALL_CHANGE_LENGTH; i++) {

      patron_led = 1;
      leds_configuracion();
      delay(50);
      patron_led = 2;
      leds_configuracion();
      delay(50);
      patron_led = 3;
      leds_configuracion();
      delay(50);
      patron_led = 3;
      leds_configuracion();
      delay(50);
      patron_led = 2;
      leds_configuracion();
      delay(50);
      patron_led = 1;
      leds_configuracion();
      delay(50);
      patron_led = 0;
      leds_configuracion();
      Serial.println(REFERENCE_WALL_CHANGE_MILLIS[i]);
    }
  }
  return configuracion_lista;
}

void leds_configuracion() {
  switch (patron_led) {
    case 0:
      digitalWrite(LED_ADELANTE, LOW);
      digitalWrite(LED_DERECHA, LOW);
      digitalWrite(LED_IZQUIERDA, LOW);
      break;
    case 1:
      digitalWrite(LED_ADELANTE, LOW);
      digitalWrite(LED_DERECHA, LOW);
      digitalWrite(LED_IZQUIERDA, HIGH);
      break;
    case 2:
      digitalWrite(LED_ADELANTE, HIGH);
      digitalWrite(LED_DERECHA, LOW);
      digitalWrite(LED_IZQUIERDA, LOW);
      break;
    case 3:
      digitalWrite(LED_ADELANTE, LOW);
      digitalWrite(LED_DERECHA, HIGH);
      digitalWrite(LED_IZQUIERDA, LOW);
      break;
    case 4:
      digitalWrite(LED_ADELANTE, LOW);
      digitalWrite(LED_DERECHA, HIGH);
      digitalWrite(LED_IZQUIERDA, HIGH);
      break;
    case 5:
      digitalWrite(LED_ADELANTE, HIGH);
      digitalWrite(LED_DERECHA, HIGH);
      digitalWrite(LED_IZQUIERDA, HIGH);
      break;
    case 6:
      digitalWrite(LED_ADELANTE, LOW);
      digitalWrite(LED_DERECHA, HIGH);
      digitalWrite(LED_IZQUIERDA, HIGH);
      break;
    case 7:
      digitalWrite(LED_ADELANTE, LOW);
      digitalWrite(LED_DERECHA, HIGH);
      digitalWrite(LED_IZQUIERDA, LOW);
      break;
    case 8:
      digitalWrite(LED_ADELANTE, HIGH);
      digitalWrite(LED_DERECHA, LOW);
      digitalWrite(LED_IZQUIERDA, LOW);
      break;
    case 9:
      digitalWrite(LED_ADELANTE, HIGH);
      digitalWrite(LED_DERECHA, LOW);
      digitalWrite(LED_IZQUIERDA, LOW);
      break;

    default:
      break;
  }
}