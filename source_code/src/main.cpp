#include "debug.h"
#include "motores.h"
#include "pines.h"
#include "sensores.h"
#include <Arduino.h>

//////////////////////
// Indica los millis a partir de los cuales realiza un cambio de pared de
// referencia (0 -> sin cambio)
//////////////////////

#define REFERENCE_WALL_CHANGE_LENGTH 0 // numero de cambios
#define MILLIS_REFERENCE_WALL_CHANGE_1 3000U
#define MILLIS_REFERENCE_WALL_CHANGE_2 MILLIS_REFERENCE_WALL_CHANGE_1 + 15000U
#define MILLIS_REFERENCE_WALL_CHANGE_3 MILLIS_REFERENCE_WALL_CHANGE_2 + 15000U
#define MILLIS_REFERENCE_WALL_CHANGE_4 MILLIS_REFERENCE_WALL_CHANGE_3 + 15000U
#define MILLIS_REFERENCE_WALL_CHANGE_5 MILLIS_REFERENCE_WALL_CHANGE_4 + 15000U

#define DETECCION_FRONTAL 180

#define DINAMICO true

bool run = false;
int objetivo_D;
int objetivo_I;
int error = 0;
int velBase = 180;

float p = 0;
float d = 0;
float kp = 0.3;
float ki = 0;
float kd = 40;
float kf = 0.8; // constante que determina cuanto afecta el sensor frontal para los giros dinamicos
int sumError = 0;
int ultError = 0;
int correccion = 0;
bool frontal = false;

bool start = false;
bool mano_izquierda = false;
bool mano_derecha = false;

int valor_sensor_lateral_izquierdo = 0;
int valor_sensor_lateral_derecho = 0;

bool debug = false;

unsigned long startedMillis = 0;

unsigned long REFERENCE_WALL_CHANGE_MILLIS[] = {
    MILLIS_REFERENCE_WALL_CHANGE_1, MILLIS_REFERENCE_WALL_CHANGE_2,
    MILLIS_REFERENCE_WALL_CHANGE_3, MILLIS_REFERENCE_WALL_CHANGE_4,
    MILLIS_REFERENCE_WALL_CHANGE_5};

bool REFERENCE_WALL_CHANGE_DONE[] = {false, false, false, false, false};

bool iniciado = false;

/////////////////////////////
////     FUNCIONES      /////
/////////////////////////////

void giros_dinamicos() {
  if (mano_izquierda == true) {
    error -= sensor2_analog() * kf;
  }
  if (mano_derecha == true) {
    error += sensor2_analog() * kf;
  }
}

void check_reference_wall_change() {
  for (int i = 0; i < REFERENCE_WALL_CHANGE_LENGTH; i++) {
    if (!REFERENCE_WALL_CHANGE_DONE[i] &&
        REFERENCE_WALL_CHANGE_MILLIS[i] != 0 &&
        millis() > startedMillis + REFERENCE_WALL_CHANGE_MILLIS[i]) {
      mano_izquierda = !mano_izquierda;
      mano_derecha = !mano_derecha;
      REFERENCE_WALL_CHANGE_DONE[i] = true;
    }
  }
  if (mano_izquierda) {
    digitalWrite(LED_IZQUIERDA, HIGH);
  } else {
    digitalWrite(LED_IZQUIERDA, LOW);
  }
}

/////////////////////////////
////        SETUP       /////
/////////////////////////////
void setup() {
  inicializar_pines();
  if (!digitalRead(BOTON_D) || !digitalRead(BOTON_I)) {
    debug = true;
    while (!digitalRead(BOTON_D) || !digitalRead(BOTON_I)) {
    }
  } else {
    inicializar_motores();
    Serial.begin(115200);
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

  filtro_sensores();

  if (!iniciado && !start) {
    if (boton_D()) {
      iniciado = true;
      mano_izquierda = true;
      digitalWrite(LED_DERECHA, HIGH);
      delay(5000);
      objetivo_I = analogRead(S_PARED_3);
      objetivo_D = analogRead(S_PARED_1);
      digitalWrite(LED_DERECHA, LOW);
    } else if (boton_I()) {
      iniciado = true;
      mano_derecha = true;
      digitalWrite(LED_IZQUIERDA, HIGH);
      delay(5000);
      objetivo_D = analogRead(S_PARED_1);
      objetivo_I = analogRead(S_PARED_3);
      digitalWrite(LED_IZQUIERDA, LOW);
    }
  } else if (!start && iniciado) {
    if (sensor2_analog() >= 450) {
      start = true;
      while (analogRead(S_PARED_2) >= 450) { // Este while es para hacer parpadear el led mientras le das la orden de arranque
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
    }
  } else if (start) {

    //////////////////////////////////////////////
    ////         TODO EL CODIGO!!!!          /////
    //////////////////////////////////////////////
    check_reference_wall_change();

    if (mano_izquierda == true) {
      error = objetivo_I - sensor3_analog();
      if (frontal && !DINAMICO) {
        asignacion_vel_motores(0, 0);
        delay(70);
        asignacion_vel_motores(0, -80);
        delay(238);
        asignacion_vel_motores(0, 0);
        return;
      }
    }
    if (mano_derecha == true) {
      error = objetivo_D - sensor1_analog();
      if (frontal && !DINAMICO) {
        asignacion_vel_motores(0, 0);
        delay(70);
        asignacion_vel_motores(0, 80);
        delay(250);
        asignacion_vel_motores(0, 0);
        return;
      }
    }
    if (DINAMICO && frontal) {
      giros_dinamicos(); // Añadir lectura delantera al error para tener giros dinamicos
    }

    p = kp * error;
    d = kd * (error - ultError);
    ultError = error;
    correccion = p + d;
    if (mano_derecha == true) {
      correccion = -correccion;
    }
    asignacion_vel_motores(velBase, correccion);
  } else {
    asignacion_vel_motores(0, 0);
  }

  //////////////////////////////////////////////
  ////     Y HASTA AQUI EL CODIGO!!!!      /////
  //////////////////////////////////////////////
}
