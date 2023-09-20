#include "algoritmo.h"
#include "debug.h"
#include "motores.h"
#include "pines.h"
#include "sensores.h"
#include <Arduino.h>

//////////////////////
// Indica los millis a partir de los cuales realiza un cambio de pared de
// referencia (0 -> sin cambio)
//////////////////////

#define DERECHA 0
#define IZQUIERDA 1

#define DETECCION_FRONTAL 500
#define TIEMPO_FILTRO 20
#define DINAMICO false

bool run = false;
int objetivo_D;
int objetivo_I;
int error = 0;
int velBase = 500;

bool mano = false;

float p = 0;
float d = 0;
float kp = 0.03;
float ki = 0;
float kd = 0;
float kf = 0.8; // constante que determina cuanto afecta el sensor frontal para los giros dinamicos
int sumError = 0;
int ultError = 0;
int correccion = 0;
bool frontal = false;
float millis_PID = 0;
float micros_filtro = 0;

bool start = false;

bool parpadeo_led_D = false;
bool parpadeo_led_I = false;

bool debug = false;

unsigned long startedMillis = 0;

bool iniciado = false;

/////////////////////////////
////     FUNCIONES      /////
/////////////////////////////

/////////////////////////////
////        SETUP       /////
/////////////////////////////
void setup() {
  inicializar_pines();
  Serial.begin(115200);
  if (!digitalRead(BOTON_D) || !digitalRead(BOTON_I)) {
    debug = true;
    while (!digitalRead(BOTON_D) || !digitalRead(BOTON_I)) {
    }
  } else {
    inicializar_motores();
  }
}
/////////////////////////////
////        LOOP        /////
/////////////////////////////
void loop() {

  if (debug) {
    debug_inicio();
    return;
  }
  // prints_sensores();
  // return;

  if (!iniciado && !start) {

    if (boton_D()) {

      iniciado = true;
      mano = DERECHA;
      digitalWrite(LED_DERECHA, HIGH);

      for (int i = 0; i < TIEMPO_FILTRO; i++) {
        filtro_sensores();
        digitalWrite(LED_DERECHA, !parpadeo_led_D);
        delay(3000 / TIEMPO_FILTRO);
      }

      objetivo_I = sensor3_analog();
      objetivo_D = sensor1_analog();
      digitalWrite(LED_DERECHA, LOW);

    } else if (boton_I()) {

      iniciado = true;
      mano = IZQUIERDA;
      digitalWrite(LED_IZQUIERDA, HIGH);

      for (int i = 0; i < TIEMPO_FILTRO; i++) {
        filtro_sensores();
        digitalWrite(LED_IZQUIERDA, !parpadeo_led_I);
        delay(3000 / TIEMPO_FILTRO);
      }

      objetivo_D = sensor1_analog();
      objetivo_I = sensor3_analog();
      digitalWrite(LED_IZQUIERDA, LOW);
    }
  } else if (!start && iniciado) {
    filtro_sensores();

    if (sensor2()) {
      start = true;
      while (sensor2()) { // Este while es para hacer parpadear el led mientras le das la orden de arranque
        filtro_sensores();
        digitalWrite(LED_ADELANTE, HIGH);
        digitalWrite(LED_DERECHA, HIGH);
        digitalWrite(LED_IZQUIERDA, HIGH);
        delay(50);
        digitalWrite(LED_ADELANTE, LOW);
        digitalWrite(LED_DERECHA, LOW);
        digitalWrite(LED_IZQUIERDA, LOW);
        delay(50);
      }
      digitalWrite(LED_ADELANTE, LOW);
      digitalWrite(LED_DERECHA, LOW);
      digitalWrite(LED_IZQUIERDA, LOW);
      delay(700);
      startedMillis = millis();
      millis_PID = millis();
      micros_filtro = micros();
    }
  }

  //////////////////////////////////////////////
  ////         TODO EL CODIGO!!!!          /////
  //////////////////////////////////////////////
  if (start) {

    if (micros() - micros_filtro > (1000 / TIEMPO_FILTRO)) {
      filtro_sensores();
      micros_filtro = micros();
    }

    if (millis() - millis_PID > 1) {
      digitalWrite(LED_ADELANTE, LOW);
      check_reference_wall_change(startedMillis, mano);

      if (sensor2_analog() > DETECCION_FRONTAL){
        frontal = true;
      }else{
        frontal = false;
      }

      if (mano == IZQUIERDA) {
        error = objetivo_I - sensor3_analog();
        if (frontal && !DINAMICO) {
          asignacion_vel_motores(0, 0);
          delay(70);
          asignacion_vel_motores(0, -300);
          delay(200);
          asignacion_vel_motores(0, 0);
          return;
        }
      }
      if (mano == DERECHA) {
        error = objetivo_D - sensor1_analog();
        if (frontal && !DINAMICO) {
          asignacion_vel_motores(0, 0);
          delay(70);
          asignacion_vel_motores(0, 300);
          delay(200);
          asignacion_vel_motores(0, 0);
          return;
        }
      }
      if (DINAMICO && frontal) { // Añadir lectura delantera al error para tener giros dinamicos
        if (mano == IZQUIERDA) {
          error -= sensor2_analog() * kf;
        }
        if (mano == DERECHA) {
          error += sensor2_analog() * kf;
        };
      }

      p = kp * error;
      d = kd * (error - ultError);
      ultError = error;
      correccion = p + d;
      if (mano == DERECHA) {
        correccion = -correccion;
      }
      asignacion_vel_motores(velBase, correccion);

      millis_PID = millis();
    }
  } else {
    digitalWrite(LED_ADELANTE, HIGH);
    asignacion_vel_motores(0, 0);
  }
}
//////////////////////////////////////////////
////     Y HASTA AQUI EL CODIGO!!!!      /////
//////////////////////////////////////////////
