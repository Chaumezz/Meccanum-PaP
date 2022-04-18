#include <AccelStepper.h>
#define ENB_LBW 24
#define ENB_LFW 30
#define ENB_RBW 38
#define ENB_RFW 56
#define MAX_SPEED  1800
#define SIDE_SPEED 2200
#define TURN_SPEED 2000
#define MIN_SPEED     0
#define MAX_DIST  10000
#define ACCEL      2000
#define led 4

//     En este caso estoy usando una RAMPS 1.3, mapeo de pines al canto, el modulo BT esta en Serial2, rX 16 y tX 17
AccelStepper LeftBackWheel(1, 26, 28);   // (Type:driver, STEP, DIR) - Stepper1 ENB_24
AccelStepper LeftFrontWheel(1, 36, 34);  // Stepper2                            ENB_30
AccelStepper RightBackWheel(1, 54, 55);  // Stepper3                            ENB_38
AccelStepper RightFrontWheel(1, 60, 61); // Stepper4                            ENB_56

//                                                     Variables de datos por BT
char BluetoothData;

//                                                     Contadores para tiempos de paro
unsigned long previousMillis = 0;
unsigned long currentMillis = millis();
int cambioDir = 2000;
int paroMot = 1600;

//                                                      Variables de joystick y mezcla ruedas
int RX, RY, LX, LY;
int speed_base, speed_side, speed_turn;
int RFW_speed, RBW_speed, LFW_speed, LBW_speed;
int wheelSpeed = 1000;

void setup() {
  // Setup enablePins en la RAMPS 1.3
  LeftBackWheel.setEnablePin(ENB_LBW);
  LeftFrontWheel.setEnablePin(ENB_LFW);
  RightBackWheel.setEnablePin(ENB_RBW);
  RightFrontWheel.setEnablePin(ENB_RFW);

  LeftBackWheel.setPinsInverted(false, false, true);
  LeftFrontWheel.setPinsInverted(false, false, true);
  RightBackWheel.setPinsInverted(false, false, true);
  RightFrontWheel.setPinsInverted(false, false, true);



  Serial.begin(115200);                //   Serial debug
  Serial1.begin(115200);                 // HC 05 en Serial1
  Serial1.setTimeout(10);
  // Serial2.begin (115200);             // Nano en Serial2
  // Serial2.setTimeout (10);

  // Setup de velocidad maxima y accel.
  LeftFrontWheel.setMaxSpeed(MAX_SPEED);
  LeftBackWheel.setMaxSpeed(MAX_SPEED);
  RightFrontWheel.setMaxSpeed(MAX_SPEED);
  RightBackWheel.setMaxSpeed(MAX_SPEED);
  /*
    LeftFrontWheel.setSpeed(0);
    LeftBackWheel.setSpeed(0);
    RightFrontWheel.setSpeed(0);
    RightBackWheel.setSpeed(0);

    LeftFrontWheel.setAcceleration(ACCEL);
    LeftBackWheel.setAcceleration(ACCEL);
    RightFrontWheel.setAcceleration(ACCEL);
    RightBackWheel.setAcceleration(ACCEL);
  */
}

void loop() {
  readBluetooth ();
  mezclaMotores ();
  actualizarRuedas ();
  //  serialDebug ();
  //  bateriaEstado ();
}


void readBluetooth () {
  //                                                   Leemos Serial1

  if (Serial1.available()) {
    BluetoothData = Serial1.read();
    if (BluetoothData == 'E') {
      if (Serial1.available()) {
        BluetoothData = Serial1.read();
        if (BluetoothData == 'X') {
          RX = Serial1.parseInt();
          while (BluetoothData != '*') {
            if (Serial1.available()) {
              BluetoothData = Serial1.read();
              if (BluetoothData == 'Y') RY = Serial1.parseInt();
            }
          }
        }
      }
    }
    if (BluetoothData == 'G') {
      if (Serial1.available()) {
        BluetoothData = Serial1.read();
        if (BluetoothData == 'X') {
          LX = Serial1.parseInt();
          while (BluetoothData != '*') {
            if (Serial1.available()) {
              BluetoothData = Serial1.read(); //Get next character from bluetooth
              if (BluetoothData == 'Y') LY = Serial1.parseInt();
            }
          }
        }
      }
    }
  }
}

void mezclaMotores () {
  //                                                           Mezcla por rueda de 4 canales de los dos joysticks
  speed_base = map ( RY, -100, 100 , MAX_SPEED, -MAX_SPEED );
  speed_side = map ( RX, -100, 100 , SIDE_SPEED, -SIDE_SPEED );
  speed_turn = map ( LX, -100, 100 , TURN_SPEED, -TURN_SPEED );

  LFW_speed = (( speed_base - speed_turn ) - speed_side );
  LBW_speed = (( speed_base - speed_turn ) + speed_side );
  RFW_speed = (( speed_base + speed_turn ) + speed_side );
  RBW_speed = (( speed_base + speed_turn ) - speed_side );
}


void actualizarRuedas () {
  //                                                               Desarmamos motores en velocidad 0 en todas las ruedas
  if (LFW_speed == 0 && LBW_speed == 0 && RFW_speed == 0 && RBW_speed == 0) {
    LeftFrontWheel.disableOutputs();
    LeftBackWheel.disableOutputs();
    RightFrontWheel.disableOutputs();
    RightBackWheel.disableOutputs();
  } else {
    LeftFrontWheel.enableOutputs();
    LeftBackWheel.enableOutputs();
    RightFrontWheel.enableOutputs();
    RightBackWheel.enableOutputs();
  }


  //                                                                  Escribimos valores en motores
  LeftFrontWheel.setSpeed(LFW_speed);
  LeftBackWheel.setSpeed(LBW_speed);
  RightFrontWheel.setSpeed(RFW_speed);
  RightBackWheel.setSpeed(RBW_speed);

  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
  /*
    LeftFrontWheel.run();
    LeftBackWheel.run();
    RightFrontWheel.run();
    RightBackWheel.run();*/
}


void serialDebug () {
  Serial.print (LFW_speed); Serial.print ("   "); Serial.print (LBW_speed); Serial.print ("   "); Serial.print (RFW_speed); Serial.print ("   "); Serial.println (RBW_speed);
}


void bateriaEstado () {
  // Monitorea la bateria
  int sensorValue = analogRead(A3);
  float voltage = sensorValue * (5.0 / 1023.00) * 3;
  Serial.println(voltage);
  // Si el voltage esta debajo de 7,0V enciende LED rojo
  if (voltage < 7.00) {
    digitalWrite(led, HIGH);
  }
  else {
    digitalWrite(led, LOW);
  }
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
