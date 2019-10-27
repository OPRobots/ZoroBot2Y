#include <Wire.h>
#include <PIDfromBT.h> // Calibracion de BT desde App Android PIDfromBT


#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

int calibracion_z = 0;


#define BTN_1 A3
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

float kp = 10;
float ki = 0;
float kd = 0;
int velBase = 0;
short ultimoError  = 0;
long ultimoMillis  = 0;
bool run = false;
PIDfromBT pid_calibrate(&kp, &ki, &kd, &velBase, DEBUG);

void setup()
{
  Wire.begin();
  Serial.begin(9600);

  pinMode(MOTOR_DERECHO_ADELANTE, OUTPUT);
  pinMode(MOTOR_DERECHO_ATRAS, OUTPUT);
  pinMode(MOTOR_IZQUIERDO_ADELANTE, OUTPUT);
  pinMode(MOTOR_IZQUIERDO_ATRAS, OUTPUT);

  pinMode(MOTOR_DERECHO_PWM, OUTPUT);
  pinMode(MOTOR_IZQUIERDO_PWM, OUTPUT);
  pinMode(MOTOR_RUN, OUTPUT);
  digitalWrite(MOTOR_DERECHO_ADELANTE, LOW);
  digitalWrite(MOTOR_DERECHO_ATRAS, LOW);
  digitalWrite(MOTOR_IZQUIERDO_ADELANTE, LOW);
  digitalWrite(MOTOR_IZQUIERDO_ATRAS, LOW);
  digitalWrite(MOTOR_RUN, HIGH);

  digitalWrite(MOTOR_DERECHO_PWM, LOW);
  digitalWrite(MOTOR_IZQUIERDO_PWM, LOW);

  pinMode(BTN_1, INPUT_PULLUP);

  // Configurar giroscopio
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
  // Configurar magnetometro
  // I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
  //  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);


  int sum_z = 0;
  for(int i = 0; i < 20;i++){
    sum_z += lectura_z();
    delay(10);
  }
  calibracion_z = sum_z/20;
}

long last_millis = 0;
float angle = 0;

void loop()
{
  pid_calibrate.update();

  if(last_millis == 0){
    last_millis = micros();
  }
  // if((micros()-last_millis) > 5){
  float z = lectura_z();
  angle +=1.05* z * (micros()-last_millis) / 1000000.0;
  last_millis = micros();
  if(!digitalRead(BTN_1)){
    //   Serial.println(angle);
  }
  // }
  set_speed(pid_grados());
  // lectura_mag();

}

float pid_grados() {
  float p= 0;
  double i = 0;
  float d = 0;
  float error = angle;

  p = kp * error;
  d = kd * ((error - ultimoError) / (micros() - ultimoMillis));
  ultimoMillis = micros();
  ultimoError = error;
  return p+d;
}

void set_speed(float correccion) {
  int velI = velBase + correccion ;//+ MOTOR_IZQUIERDO_OFFSET;
  int velD = velBase - correccion ;//+ MOTOR_DERECHO_OFFSET;

  int pinD = MOTOR_DERECHO_ADELANTE;
  int pinI = MOTOR_IZQUIERDO_ADELANTE;
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

  analogWrite(MOTOR_DERECHO_PWM   , velD);
  analogWrite(MOTOR_IZQUIERDO_PWM , velI);
}

int lectura_z(){
  // ---  Lectura acelerometro y giroscopio ---
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
  int z = ((Buf[12] << 8 | Buf[13]) - calibracion_z)/16.4;
  /*
  
  switch (scale){
        case BITS_FS_250DPS:   gyro_divider = 131;  break;
        case BITS_FS_500DPS:   gyro_divider = 65.5; break;
        case BITS_FS_1000DPS:  gyro_divider = 32.8; break;
        case BITS_FS_2000DPS:  gyro_divider = 16.4; break;   
    }


     switch (scale){
        case BITS_FS_2G:
            acc_divider=16384;
        break;
        case BITS_FS_4G:
            acc_divider=8192;
        break;
        case BITS_FS_8G:
            acc_divider=4096;
        break;
        case BITS_FS_16G:
            acc_divider=2048;
        break;   
    }

   */
  return (abs(z) > 1)? z: 0;
}

//Funcion auxiliar lectura
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
  Data[index++] = Wire.read();
}
// Funcion auxiliar de escritura
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}
