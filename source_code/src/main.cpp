#include <Arduino.h>

///////////////////
// PINES MOTORES //
///////////////////
#define MOTOR_DERECHO_ADELANTE 9
#define MOTOR_DERECHO_ATRAS 10
#define MOTOR_DERECHO_PWM 11
#define MOTOR_IZQUIERDO_ADELANTE 7
#define MOTOR_IZQUIERDO_ATRAS 6
#define MOTOR_IZQUIERDO_PWM 5
#define MOTOR_RUN 8

////////////////////
// PINES SENSORES //
////////////////////
#define SENSOR_LATERAL_IZQUIERDO A7
#define SENSOR_LATERAL_DERECHO A1
#define SENSOR_FRONTAL A6
#define SENSOR_START A0

//////////////////////
// VARIABLES NEUTRAS //
//////////////////////
#define LED_PIN 13

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
int SENSOR_LATERAL;
int OBJETIVO_D;
int OBJETIVO_I;
int error = 0;
int velBase = 180;
float kp = 0.3;
float ki = 0;
float kd = 40;
float kf = 0.8; // constante que determina cuanto afecta el sensor frontal para
                // los giros dinamicos
int sumError;
int ultError;
int correccion;
bool frontal = false;

bool start = false;
bool mano_izquierda = false;
bool mano_derecha = false;

int valor_sensor_frontal = 0;
int valor_sensor_lateral_izquierdo = 0;
int valor_sensor_lateral_derecho = 0;

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

void prints_sensores() {
  Serial.print(valor_sensor_frontal);
  Serial.print(" ");
  Serial.print(valor_sensor_lateral_izquierdo);
  Serial.print(" ");
  Serial.print(valor_sensor_lateral_derecho);
  Serial.print(" ");
  Serial.print(OBJETIVO_D);
  Serial.print(" ");
  Serial.print(OBJETIVO_I);
  Serial.println(" ");
}

void giros_dinamicos() {
  if (mano_izquierda == true) {
    error -= valor_sensor_frontal * kf;
  }
  if (mano_derecha == true) {
    error += valor_sensor_frontal * kf;
  }
}

void filtro_sensores() {
  valor_sensor_frontal = 0;
  valor_sensor_lateral_izquierdo = 0;
  valor_sensor_lateral_derecho = 0;
  for (int i = 0; i < 5; i++) {
    valor_sensor_frontal = valor_sensor_frontal + analogRead(SENSOR_FRONTAL);
    valor_sensor_lateral_izquierdo =
        valor_sensor_lateral_izquierdo + analogRead(SENSOR_LATERAL_IZQUIERDO);
    valor_sensor_lateral_derecho =
        valor_sensor_lateral_derecho + analogRead(SENSOR_LATERAL_DERECHO);
  }

  valor_sensor_frontal = valor_sensor_frontal / 5;
  valor_sensor_lateral_izquierdo = valor_sensor_lateral_izquierdo / 5;
  valor_sensor_lateral_derecho = valor_sensor_lateral_derecho / 5;
}

void set_speed(float velBase, float correccion) {
  int pinD = MOTOR_DERECHO_ADELANTE;
  int pinI = MOTOR_IZQUIERDO_ADELANTE;
  int velI = velBase - correccion;
  int velD = velBase + correccion;

  // Limitar velocidad del motor derecho y selecciona la dirección.
  if (velD > 255) {
    velD = 255;
    pinD = MOTOR_DERECHO_ADELANTE;
  } else if (velD < 0) {
    velD = abs(velD);
    if (velD > 255) {
      velD = 255;
    }
    pinD = MOTOR_DERECHO_ATRAS;
  }

  // Limitar velocidad del motor izquierdo y selecciona la dirección.
  if (velI > 255) {
    velI = 255;
    pinI = MOTOR_IZQUIERDO_ADELANTE;
  } else if (velI < 0) {
    velI = abs(velI);
    if (velI > 255) {
      velI = 255;
    }
    pinI = MOTOR_IZQUIERDO_ATRAS;
  }

  if (!run) {
    run = true;
    digitalWrite(MOTOR_RUN, HIGH);
  }
  digitalWrite(MOTOR_DERECHO_ADELANTE, LOW);
  digitalWrite(MOTOR_DERECHO_ATRAS, LOW);
  digitalWrite(MOTOR_IZQUIERDO_ADELANTE, LOW);
  digitalWrite(MOTOR_IZQUIERDO_ATRAS, LOW);

  digitalWrite(pinD, HIGH);
  digitalWrite(pinI, HIGH);

  analogWrite(MOTOR_DERECHO_PWM, velD);
  analogWrite(MOTOR_IZQUIERDO_PWM, velI);
}

void init_motores() {
  // Inicialización de pines
  pinMode(MOTOR_DERECHO_ADELANTE, OUTPUT);
  pinMode(MOTOR_DERECHO_ATRAS, OUTPUT);
  pinMode(MOTOR_IZQUIERDO_ADELANTE, OUTPUT);
  pinMode(MOTOR_IZQUIERDO_ATRAS, OUTPUT);
  pinMode(MOTOR_DERECHO_PWM, OUTPUT);
  pinMode(MOTOR_IZQUIERDO_PWM, OUTPUT);
  pinMode(MOTOR_RUN, OUTPUT);

  // Estado inicial; parado
  digitalWrite(MOTOR_DERECHO_ADELANTE, LOW);
  digitalWrite(MOTOR_DERECHO_ATRAS, LOW);
  digitalWrite(MOTOR_IZQUIERDO_ADELANTE, LOW);
  digitalWrite(MOTOR_IZQUIERDO_ATRAS, LOW);
  digitalWrite(MOTOR_RUN, HIGH);
  digitalWrite(MOTOR_DERECHO_PWM, LOW);
  digitalWrite(MOTOR_IZQUIERDO_PWM, LOW);
}

void init_sensores() {
  pinMode(SENSOR_LATERAL_IZQUIERDO, INPUT);
  pinMode(SENSOR_LATERAL_DERECHO, INPUT);
  pinMode(SENSOR_FRONTAL, INPUT);
  pinMode(SENSOR_START, INPUT);
}
void init_btn_led() { pinMode(LED_PIN, OUTPUT); }

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
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

/////////////////////////////
////        SETUP       /////
/////////////////////////////
void setup() {
  init_motores();
  init_sensores();
  init_btn_led();
  Serial.begin(9600);
}

/////////////////////////////
////        LOOP        /////
/////////////////////////////
void loop() {

  // prints_sensores();
  // return;

  filtro_sensores();

  if (!iniciado && !start) {
    if (valor_sensor_lateral_izquierdo >= 200) {
      iniciado = true;
      mano_izquierda = true;
      digitalWrite(LED_PIN, HIGH);
      delay(5000);
      OBJETIVO_I = analogRead(SENSOR_LATERAL_IZQUIERDO);
      OBJETIVO_D = analogRead(SENSOR_LATERAL_DERECHO);
      digitalWrite(LED_PIN, LOW);
    } else if (valor_sensor_lateral_derecho >= 200) {
      iniciado = true;
      mano_derecha = true;
      digitalWrite(LED_PIN, HIGH);
      delay(5000);
      OBJETIVO_D = analogRead(SENSOR_LATERAL_DERECHO);
      OBJETIVO_I = analogRead(SENSOR_LATERAL_IZQUIERDO);
      digitalWrite(LED_PIN, LOW);
    }
  } else if (!start && iniciado) {
    if (valor_sensor_frontal >= 450) {
      start = true;
      while (analogRead(SENSOR_FRONTAL) >= 450) { // Este while es para hacer parpadear el led mientras le das la orden de arranque
        digitalWrite(LED_PIN, HIGH);
        delay(50);
        digitalWrite(LED_PIN, LOW);
        delay(50);
      }
      digitalWrite(LED_PIN, LOW);
      delay(700);
      startedMillis = millis();
    }
  } else if (start) {

    //////////////////////////////////////////////
    ////         TODO EL CODIGO!!!!          /////
    //////////////////////////////////////////////
    check_reference_wall_change();

    if (valor_sensor_frontal > DETECCION_FRONTAL) {

      frontal = true;

    } else {
      frontal = false;
    }

    if (mano_izquierda == true) {
      error = OBJETIVO_I - valor_sensor_lateral_izquierdo;
      if (frontal && !DINAMICO) {
        set_speed(0, 0);
        delay(70);
        set_speed(0, -80);
        delay(238);
        set_speed(0, 0);
        return;
      }
    }
    if (mano_derecha == true) {
      error = OBJETIVO_D - valor_sensor_lateral_derecho;
      if (frontal && !DINAMICO) {
        set_speed(0, 0);
        delay(70);
        set_speed(0, 80);
        delay(250);
        set_speed(0, 0);
        return;
      }
    }
    if (DINAMICO && frontal) {
      giros_dinamicos(); // Añadir lectura delantera al error para tener giros dinamicos
    }

    float p, d;
    p = kp * error;
    d = kd * (error - ultError);
    ultError = error;
    correccion = p + d;
    if (mano_derecha == true) {
      correccion = -correccion;
    }
    set_speed(velBase, correccion);
  } else {
    set_speed(0, 0);
  }

  //////////////////////////////////////////////
  ////     Y HASTA AQUI EL CODIGO!!!!      /////
  //////////////////////////////////////////////
}
