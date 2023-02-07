#include "Arduino.h"
#include "ModulesCarDrivers.hpp"
#include "TimerFive.h"

/*-----------------DEFINICIONES PARA PARÁMETROS----------------------*/
// Pines de conexionado
//-Motores-
#define pinENA 5
#define pinENB 6
#define pinIN1 2
#define pinIN2 4
#define pinIN3 8
#define pinIN4 7
//-Ultrasonidos-
#define EchoPinA 13
#define TriggerPinA 12
#define EchoPinB 14
#define TriggerPinB 15
//-Encoders-
#define pinEncoder1 20;
#define pinEncoder2 21;
#define npolos 8 // Número de cambios de tensión correspondientes a una vuelta (doble de ranuras del encoder)
#define reductora 48
#define refreshRateEncoders 50UL // en milisengudos
//-Controlador-
#define puntoEquilibrio 50  // PWM mínimo que se aplica para estar cercanos al movimiento del robot
#define referenciaInicial 50 // referencia inicial de control de velocidad
#define Kp 1
#define Ki 0.001
#define Kd 0
#define Kp_lr 100
#define Ki_lr 0.000001
#define Kd_lr 0

/*---------------VARIABLES GLOBALES-------------------*/
int ticksA = 0;
int ticksB = 0;
float wA = 0, wB = 0;
double distanciaA = 0, distanciaB = 0;
const int ENC1 = pinEncoder1;
const int ENC2 = pinEncoder2;
movimientoCoche motorA(pinENA, pinIN1, pinIN2);
movimientoCoche motorB(pinENB, pinIN3, pinIN4);
/*------------------CABECERAS DE FUNCIONES/SUBRUTINAS  Y VARIABLES GLOBALES ASOCIADAS----------------------------------------*/
void aplicaSignal(movimientoCoche &motor, int signalControl, float RefVelAplicada);
void comprueba_cambio_sentido(float RefVel, float *RefVel_aplicada, movimientoCoche &motor);
void ISR_calculateAngSpeed();
void ISR_incrTicksA();
void ISR_incrTicksB();
int sign(float a);

/*========================================================================================================================*/

void setup()
{
  // Configuración pines ultrasonidos
  pinMode(TriggerPinA, OUTPUT);
  pinMode(EchoPinA, INPUT);
  pinMode(TriggerPinB, OUTPUT);
  pinMode(EchoPinB, INPUT);

  // Configuración pines motor
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinENA, OUTPUT);
  pinMode(pinENB, OUTPUT);

  // Configuración pines Encoder como interrupciones
  Timer5.initialize(refreshRateEncoders * 1000);
  Timer5.attachInterrupt(ISR_calculateAngSpeed);
  attachInterrupt(digitalPinToInterrupt(ENC1), ISR_incrTicksA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2), ISR_incrTicksB, CHANGE);

  // Configuración puerto bluetooth para telemetría
  Serial1.begin(9600);

  // Configuración puerto usb para depuración
  Serial.begin(9600);
}

void loop()
{
  static bool powerON = false;
  static Bluetooth moduloBluetooth;
  static float RefVelA = referenciaInicial;
  static float RefVelA_aplicada = referenciaInicial;
  static float RefVelB = referenciaInicial;
  static float RefVelB_aplicada = referenciaInicial;
  static Controlador LineaRecta(Kp_lr, Ki_lr, Kd_lr, -5, 5);
  static Controlador ControlVelocidadA(Kp, Ki, Kd, -255 + puntoEquilibrio, 255 - puntoEquilibrio);
  static Controlador ControlVelocidadB(Kp, Ki, Kd, -255 + puntoEquilibrio, 255 - puntoEquilibrio);
  static float deltaVel;
  static int PWM_A, PWM_B;

  do
  {
    moduloBluetooth.recibeDatosBluetooth(&RefVelA, &RefVelB, &powerON, ControlVelocidadA, ControlVelocidadB, motorA, motorB); // leer si se ha recibido alguna orden
  } while (!powerON);

  // Comprobamos si el signo de la referencia ha cambiado
  comprueba_cambio_sentido(RefVelA, &RefVelA_aplicada, motorA);
  comprueba_cambio_sentido(RefVelB, &RefVelB_aplicada, motorB);

  deltaVel = LineaRecta.calcularSalida(0-(distanciaA-distanciaB));

  if(RefVelA > 0){
  if(deltaVel<0)
  {
    RefVelB_aplicada -= deltaVel;
  }
  else
  {
    RefVelA_aplicada += deltaVel;
  }
  } else{
    if(deltaVel<0)
  {
    RefVelB_aplicada += deltaVel;
  }
  else
  {
    RefVelA_aplicada -= deltaVel;
  }
}

  PWM_A = ControlVelocidadA.calcularSalida(RefVelA_aplicada - wA);
  if (RefVelA_aplicada > 0)
  {
    PWM_A += puntoEquilibrio;
    motorA.sentidoGiro = 1;
  }
  else if (RefVelA_aplicada < 0)
  {
    PWM_A -= puntoEquilibrio;
    motorA.sentidoGiro = -1;
  }

  PWM_B = ControlVelocidadB.calcularSalida(RefVelB_aplicada - wB);
  if (RefVelB_aplicada > 0)
  {
    PWM_B += puntoEquilibrio;
    motorB.sentidoGiro = 1;
  }
  else if (RefVelB_aplicada < 0)
  {
    PWM_B -= puntoEquilibrio;
    motorB.sentidoGiro = -1;
  }

  aplicaSignal(motorA, PWM_A,RefVelA_aplicada);
  aplicaSignal(motorB, PWM_B,RefVelB_aplicada);
  moduloBluetooth.DebugValuesBluetooth(wA, wB, RefVelA_aplicada, RefVelB_aplicada, PWM_A, PWM_B);
}

void ISR_incrTicksA()
{
  ticksA++;
}

void ISR_incrTicksB()
{
  ticksB++;
}

void ISR_calculateAngSpeed()
{

  if (ticksA == 0) // si la rueda A está parada
  {
    motorA.sentidoGiro = 0;
    wA = 0;
  }
  else
  {
    if (motorA.sentidoGiro == 1)
    {
      wA = ((float)ticksA * 1000.0 * 60.0) / ((float)npolos * (float)reductora * refreshRateEncoders); // número de vueltas/tiempo entre medición [rps] -> *60 [rpm]
    }
    else
    {
      wA = -((float)ticksA * 1000.0 * 60.0) / ((float)npolos * (float)reductora * refreshRateEncoders);
    }
  }
  ticksA = 0;
  distanciaA += wA*0.0325*0.05*2.0*PI/60.0; //0.0325 de radio de la rueda y 0.05 seg exactos ya que está dentro de la rutina de interrupción del timer

  if (ticksB == 0) // si la rueda B está parada
  {
    motorB.sentidoGiro = 0;
    wB = 0;
  }
  else
  {
    if (motorB.sentidoGiro == 1)
    {
      wB = ((float)ticksB * 1000.0 * 60.0) / ((float)npolos * (float)reductora * refreshRateEncoders); // con ticks=1 en esta fórmula obtnemos nuestra resolución, que como se aprecia depende del tiempo de refresco
    }
    else
    {
      wB = -((float)ticksB * 1000.0 * 60.0) / ((float)npolos * (float)reductora * refreshRateEncoders);
    }
  }
  ticksB = 0;
  distanciaB += wB*0.0325*0.05*2.0*PI/60.0; 
}

void aplicaSignal(movimientoCoche &motor, int signalControl, float RefVelAplicada)
{
  if (RefVelAplicada == 0) // valor reservado para frenos
  {
    motor.stop();
  }
  else if (signalControl > 0)
  {
    motor.forward(signalControl);
  }
  else if (signalControl < 0)
  {
    motor.backward(abs(signalControl));
  }
}

void comprueba_cambio_sentido(float RefVel, float *RefVel_aplicada, movimientoCoche &motor)
{
  if (sign(RefVel) != sign(*RefVel_aplicada))
  {
    if (motor.sentidoGiro == 0)
    {
      *RefVel_aplicada = RefVel;
    }
    else
    {
      *RefVel_aplicada = 0;
    }
  }
  else 
  {
    *RefVel_aplicada = RefVel;
  }
}

int sign(float a)
{
  if (a > 0)
    return 1;
  else if (a < 0)
    return -1;
  else
    return 0;
}
