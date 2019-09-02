#include "Arduino.h"
#include "NovoQTR.h"

/*----------------*DEFAUT DO MOTOR*----------------*/
//motor A conectado entre A01 e A02
//motor B conectado entre B01 e B02
int STBY = 10; //standby

// --- Seleção dos Motores ---
//Seleção do Motor 1
//Motor A
int PWMA = 9; //controle de velocidade
int AIN1 = 7; //Direção
int AIN2 = 6; //Direção
//Seleção do Motor 2
//Motor B
int PWMB = 8; //controle de velocidade
int BIN1 = 5; //Direção
int BIN2 = 4; //Direção

/*----------------*FUNÇÕES DOS MOTORES *----------------*/
void move(int motor, int speed, int direction);

/*----------------*DEFAULT DOS ENCOLDERS *----------------*/
#define RH_ENCODER_A 2
#define RH_ENCODER_B 3
#define LH_ENCODER_A 20
#define LH_ENCODER_B 21
volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;

/*FUNÇÕES DOS ENCOLDERS */

void leftEncoderEvent();
void stop();
// encoder event for the interrupt call
void rightEncoderEvent();
void calibra();
double controle(unsigned int pos);
void leituraVelocidade();

/*DEFAUT DOS  SENSORES*/
#define NUM_SENSORS 8                  // number of sensors used
#define TIMEOUT 2500                   // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN QTR_NO_EMITTER_PIN // emitter is controlled by digital pin 52
#define maxspeedL 160
#define maxspeedR 165
//Objetos Dos Sensores

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively

QTRSensorsRC qtrrc((unsigned char[]){38, 40, 42, 44, 46, 48, 50,52},
                   NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// --- Variáveis Globais ---


int RightBaseSpeed = 105;
int LeftBaseSpeed = 100;

int Error = 0;
long derivativo=0;
long integrativo=0;
int LastError = 0;
unsigned long lasttime=0;


int RightMaxSpeed = maxspeedR;
int LeftMaxSpeed = maxspeedL;
int RigthRPM;
int LeftRPM;



bool white = LOW;

double Kp = 0.008;
double Ki = 0;
double Kd = 0;
double pot;

//ALGORITIMOS PARA CONTROLE DE GANHO
void ControledeGanhoBT();
void ControledeGanhoPot(char ganho);

void setup()
{
  delay(50);
  pinMode(13, OUTPUT);

  pinMode(STBY, INPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);

  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(20), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), rightEncoderEvent, CHANGE);
  Serial.begin(9600);
  Serial1.begin(9600);

  calibra();

  Serial1.println("Começando em o carro em...");
 
  for(int i=10; i>0;i--){ 
  Serial.println(i);
  Serial1.println(i);
  delay(1000);
  }
  Serial1.println("Já!!!");
  Serial.println("Já!!!");
}

void loop()
{
 

  if(millis()-lasttime > 500){
    
   RigthRPM=(rightCount/175)*60;
   LeftRPM=(leftCount/175)*60;
   rightCount=0;
   leftCount=0;
   lasttime=millis();
  }

  unsigned int position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, white);

  leftCount=rightCount=0;
  
  int MotorSpeed=controle(position);

  

  int RightMotorSpeed = RightBaseSpeed + MotorSpeed;

  int LeftMotorSpeed = LeftBaseSpeed - MotorSpeed;

  Serial1.print("    RightMotorSpeed:");
  Serial1.print(RightMotorSpeed);
  Serial1.print("    LeftMotorSpeed:");
  Serial1.print(LeftMotorSpeed);

  leituraVelocidade();

  if (RightMotorSpeed > RightMaxSpeed)
  {
    RightMotorSpeed = RightMaxSpeed; // prevent the motor from going beyond max speed
  }
  if (LeftMotorSpeed > LeftMaxSpeed)
  {
    LeftMotorSpeed = LeftMaxSpeed; // prevent the motor from going beyond max speed
  }
  if (RightMotorSpeed < 0)
  {
    RightMotorSpeed = 0; // keep the motor speed positive
  }
  if (LeftMotorSpeed < 0)
  {
    LeftMotorSpeed = 0; // keep the motor speed positive
  }

  move(0, LeftMotorSpeed, 1);  //Seleciona velocidade atual
  move(1, RightMotorSpeed, 0); //Seleciona velocidade atual
 
  
}

void leituraVelocidade(){
  
  Serial1.print("    RIGTH RPM:");
  Serial1.print(RigthRPM);
  Serial1.print("    LEFT RPM:");
  Serial1.println(LeftRPM);
}

void move(int motor, int speed, int direction)
{
  //Move specific motor at speed and direction
  //motor: 0 for B 1 for A
  //speed: 0 is off, and 255 is full speed
  //direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1)
  {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if (motor == 1)
  {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }
  else
  {
    digitalWrite(BIN1, inPin1);
    
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void stop()
{
  //enable standby
  digitalWrite(STBY, LOW);
}

void ControledeGanhoPot(char ganho)
{

  if (ganho == 'P')
    Kp = analogRead(pot) / 100;
  else if (ganho == 'D')
    Kd = analogRead(pot) / 100;
  else if (ganho == 'I')
    ;
  Ki = analogRead(pot) / 10000;
  //Kp = map(analogRead(Pot),0,1023,0.04,0.06);
  Serial1.print("   Ganho KP:");
  Serial1.print(Kp, 5);
  // Ultmoo GANHO DE 0,61 { melhor faixa 0,05 a 1,5}
  // Ultimo ganho  de 0.025
  Serial1.print("   Ganho KI:");
  Serial1.print(Ki, 5); //

  Serial1.print("   Ganho KD:");
  Serial1.print(Kd, 5); // 6,38
}

void leftEncoderEvent()
{
  if (digitalRead(LH_ENCODER_A) == HIGH)
  {
    if (digitalRead(LH_ENCODER_B) == LOW)
    {
      leftCount--;
    }
    else
    {
      leftCount++;
    }
  }
  else
  {
    if (digitalRead(LH_ENCODER_B) == LOW)
    {
      leftCount++;
    }
    else
    {
      leftCount--;
    }
  }
}

void rightEncoderEvent()
{
  if (digitalRead(RH_ENCODER_A) == HIGH)
  {
    if (digitalRead(RH_ENCODER_B) == LOW)
    {
      rightCount++;
    }
    else
    {
      rightCount--;
    }
  }
  else
  {
    if (digitalRead(RH_ENCODER_B) == LOW)
    {
      rightCount--;
    }
    else
    {
      rightCount++;
    }
  }
}

void ControledeGanhoBT()
{

  if (Serial1.available() > 0)
  {
    Kp = Serial1.parseFloat();
    Serial1.read();

    Ki = Serial1.parseFloat();
    Serial1.read();

    Kd = Serial1.parseFloat();
    Serial1.read();
  }
  //Kp = map(analogRead(Pot),0,1023,0.04,0.06);
  Serial1.print("   Ganho KP:");
  Serial1.print(Kp, 5);
  // Ultmoo GANHO DE 0,61 { melhor faixa 0,05 a 1,5}
  // Ultimo ganho  de 0.025
  Serial1.print("   Ganho KI:");
  Serial1.print(Ki, 5); //

  Serial1.print("   Ganho KD:");
  Serial1.print(Kd, 5); // 6,38
}

void calibra()
{
  
  Serial.println("Calibrating...");
  Serial1.println("Calibrating...");

  digitalWrite(13, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  for (int i = 0; i < 400; i++) // make the calibration take about 10 seconds
  {
    qtrrc.calibrate(); // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }

  digitalWrite(13, LOW); // turn off Arduino's LED to indicate we are through with calibration

  stop();

  // print the calibration minimum values measured when emitters were on
  Serial.println("Valores minimos:");
  Serial1.println("Valores minimos:");

  for (int i = 0; i < NUM_SENSORS; i++)
  {

    Serial1.print("Sensor");
    Serial1.print(i);
    Serial1.print(": ");
    Serial1.print(qtrrc.calibratedMinimumOn[i]);
    Serial1.print(' ');

    Serial.print("Sensor");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial1.println();

  Serial1.println("Valores maximos:");
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {

    Serial1.print("Sensor");
    Serial1.print(i);
    Serial1.print(": ");
    Serial1.print(qtrrc.calibratedMaximumOn[i]);

    Serial.print(' ');
    Serial.print("Sensor");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial1.println();
  Serial.println();

  Serial1.print(" \n Calibrition complete");
  Serial.print("Calibration complete");

  Serial1.println();
  Serial.println();
}

double controle(unsigned int pos)
{
  double P = 0;
  double I = 0;
  double D = 0;
  Error = (pos - 3500);
  Serial1.print("  Error:");
  Serial1.print(Error);

  ControledeGanhoBT();

  derivativo= (Error - LastError);
  integrativo += LastError; 
  P = Kp * Error;
  I = Ki * integrativo;
  D = Kd * derivativo;


  Serial1.print(" Erro:");
  Serial1.print(Error);
  Serial1.print('\t');

  Serial1.print(" PID:");
  Serial1.print(P+I+D);
  Serial1.print('\t');

  LastError = Error;
/*
  Serial1.print("   P:");
  Serial1.print(P);

  Serial1.print("   I:");
  Serial1.print(I);

  Serial1.print("   D:");
  Serial1.print(D);
*/
  return P+I+D;
}