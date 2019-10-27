#include <PIDfromBT.h>

///////////////////
// PINES MOTORES //
///////////////////
#define MOTOR_DERECHO_ADELANTE 9
#define MOTOR_DERECHO_ATRAS 10
#define MOTOR_DERECHO_PWM 11
#define MOTOR_IZQUIERDO_ADELANTE 6
#define MOTOR_IZQUIERDO_ATRAS 7
#define MOTOR_IZQUIERDO_PWM 5
#define MOTOR_RUN 8

////////////////////
// PINES SENSORES //
////////////////////
#define SENSOR_LATERAL_IZQUIERDO A7
#define SENSOR_FRONTAL_IZQUIERDO A6
#define SENSOR_FRONTAL_DERECHO A1
#define SENSOR_LATERAL_DERECHO A0

//////////////////////
//VARIABLES NEUTRAS //
//////////////////////
#define LED_PIN 13
#define BTN_1 A3
#define BTN_2 A2

#define DETECCION_FRONTAL 200

bool run = false;
int SENSOR_LATERAL;
int SENSOR_FRONTAL;
int OBJETIVO;
int velBase = 120;
float kp = 0.3;
float ki = 0;
float kd = 40;
int sumError;
int ultError;
int correccion;

bool mano_izquierda = false;
bool mano_derecha = false;

int valor_sensor_frontal_izquierdo = 0;
int valor_sensor_lateral_izquierdo = 0;
int valor_sensor_frontal_derecho = 0;
int valor_sensor_lateral_derecho = 0;
PIDfromBT pid_calibrate(&kp, &ki, &kd, &velBase, DEBUG);

bool iniciado = false;

void setup() {
  Serial.begin(9600);
  init_motores();
  init_sensores();
  init_btn_led();
}

void loop() {

  for (int i = 0; i < 5 ; i++) {
    if (mano_izquierda == true || (mano_izquierda == false && mano_derecha == false)) {
      valor_sensor_frontal_izquierdo = valor_sensor_frontal_izquierdo + analogRead(SENSOR_FRONTAL_IZQUIERDO);
      valor_sensor_lateral_izquierdo = valor_sensor_lateral_izquierdo + analogRead(SENSOR_LATERAL_IZQUIERDO);
    }
    if (mano_derecha == true || (mano_izquierda == false && mano_derecha == false)) {
      valor_sensor_frontal_derecho = valor_sensor_frontal_derecho + analogRead(SENSOR_FRONTAL_DERECHO);
      valor_sensor_lateral_derecho = valor_sensor_lateral_derecho + analogRead(SENSOR_LATERAL_DERECHO);
    }
  }
  if (mano_izquierda == true || (mano_izquierda == false && mano_derecha == false)) {
    valor_sensor_frontal_izquierdo = valor_sensor_frontal_izquierdo / 5;
    valor_sensor_lateral_izquierdo = valor_sensor_lateral_izquierdo / 5;
  }
  if (mano_derecha == true || (mano_izquierda == false && mano_derecha == false)) {
    valor_sensor_frontal_derecho = valor_sensor_frontal_derecho / 5;
    valor_sensor_lateral_derecho = valor_sensor_lateral_derecho / 5;
  }

  pid_calibrate.update();
  if (!iniciado) {
    if (valor_sensor_frontal_izquierdo >= 200) {

      iniciado = true;
      mano_izquierda = true;
      OBJETIVO = valor_sensor_lateral_izquierdo;
      digitalWrite(LED_PIN, HIGH);
      delay(2000);
    } else if (valor_sensor_frontal_derecho >= 200) {

      iniciado = true;
      mano_derecha = true;
      OBJETIVO = valor_sensor_lateral_derecho;
      digitalWrite(LED_PIN, HIGH);
      delay(2000);
    }
  } else {

    //////////////////////////////////////////////
    ////         TODO EL CODIGO!!!!          /////
    //////////////////////////////////////////////

    int error, frontal;

    if (mano_izquierda == true) {
      error = OBJETIVO - valor_sensor_lateral_izquierdo;
      frontal = valor_sensor_frontal_izquierdo;
      if (frontal > 350) {
        set_speed(0, 0);
        delay(70);
        set_speed(0, -80);
        delay(238);
        set_speed(0, 0);
        return;
      }
    }
    if (mano_derecha == true) {
      error = OBJETIVO - valor_sensor_lateral_derecho;
      frontal = valor_sensor_frontal_derecho;
      if (frontal > 350) {
        set_speed(0, 0);
        delay(70);
        set_speed(0, 80);
        delay(250);
        set_speed(0, 0);
        return;
      }
    }
    if (frontal > 350) {
      /*  if (mano_izquierda == true) {
          error -= frontal - 100;
        }
        if (mano_derecha == true) {
          error += frontal - 100;
        }
      */
      set_speed(100, 50);
      delay(200);
      set_speed(0, 0);
      return;
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
  }


  /* Serial.print (analogRead(s_izquierda));
    Serial.print (" ");
    Serial.print (analogRead(s_frontal_izquierda));
    Serial.print (" ");
    Serial.print (analogRead(s_frontal_derecha));
    Serial.print (" ");
    Serial.print (analogRead(s_derecha));
    Serial.print (" ");
    Serial.print (digitalRead(encoder_A_I));
    Serial.print (" ");
    Serial.print (digitalRead(encoder_A_D));
    Serial.print (" ");
    Serial.print (digitalRead(encoder_B_I));
    Serial.print (" ");
    Serial.print (digitalRead(encoder_B_D));
    Serial.print (" ");
    Serial.print (digitalRead(interruptor_1));
    Serial.print (" ");
    Serial.println (digitalRead(interruptor_2));
  */



  //////////////////////////////////////////////
  ////     Y HASTA AQUI EL CODIGO!!!!      /////
  //////////////////////////////////////////////


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
  pinMode(SENSOR_FRONTAL_IZQUIERDO, INPUT);
  pinMode(SENSOR_FRONTAL_DERECHO, INPUT);
  pinMode(SENSOR_LATERAL_DERECHO, INPUT);
}
void init_btn_led() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
}
