#include<Servo.h>

//Pines del motor 1 y 2
int PinA1 = 3;
int PinA2 = 5;
int PinB1 = 6;
int PinB2 = 9;
//Servo Motor
int PinServo1 = 7;
//Potenciometro del motor  1 y 2
int PinSensor1 = A0;
int PinSensor2 = A1;
//Potenciometro del servo
int PinSensor3 = A4;
//Potenciometro de Velocidad del motor 1 y 2
int PinVelocidad1 = A2;
int PinVelocidad2 = A3;

//objeto del servo
Servo Foco;

//Valores de Referencia
int Centro = 1024  / 2;
int Deface = 300;
int EstadoServo;
int DefaceServo = 2;


void setup() {
  pinMode(PinA1, OUTPUT);
  pinMode(PinA2, OUTPUT);
  pinMode(PinB1, OUTPUT);
  pinMode(PinB2, OUTPUT);
  Serial.begin(9600);

  int Posicion = analogRead(PinSensor3);
  EstadoServo = map(Posicion, 0, 1024, 0, 180);
  Foco.attach(PinServo1);
  Foco.write(EstadoServo);
}

void loop() {
  moverMotor(PinA1, PinA2, PinSensor1, PinVelocidad1);
  moverMotor(PinB1, PinB2, PinSensor2, PinVelocidad2);
  moverServo(Foco, PinSensor3);
  delay(100);

}

void moverServo(Servo Motor, int PinSensor) {
  int Posicion = analogRead(PinSensor);
  int Grado = map(Posicion, 0, 1024, 0, 180);
  Grado = constrain(Grado, 0, 180);
  Serial.print("G ");
  Serial.print(Grado);
  Serial.print(" E ");
  Serial.println(EstadoServo);
  if (abs(Grado - EstadoServo) >= DefaceServo) {
    EstadoServo = Grado;
    Motor.write(Grado);
  }
}

void moverMotor(int PinA, int PinB, int PinSensor, int PinVelocidad) {
  //Leer la posicion de la joystick
  int Posicion = analogRead(PinSensor);
  //Palanca de la Velocidad se trasforma de 0-1024 a 0-255
  int Velocidad = map(analogRead(PinVelocidad), 0, 1024, 0, 255);
  Velocidad = constrain(Velocidad, 0, 255);
  //Si el valor es major de mueve
  if ( Posicion > Centro + Deface) {
    analogWrite(PinA, 0);
    analogWrite(PinB, Velocidad);
    Serial.print("Derec ");
  }
  else if (Posicion < Centro - Deface) {
    analogWrite(PinA, Velocidad);
    analogWrite(PinB, 0);
    Serial.print("Izqui ");
  }
  //Parar el motor
  else {
    analogWrite(PinA, 1);
    analogWrite(PinB, 1);
    Serial.print("Parar ");

  }
  //Serial.print("V ");
  //Serial.print(Velocidad);
  //Serial.print(" P:");
  //Serial.println(Posicion);
  //delay(100);
}
