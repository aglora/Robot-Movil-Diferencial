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
#define pinEncoder1 20
#define pinEncoder2 21
#define npolos 8 // Número de cambios de tensión correspondientes a una vuelta (doble de ranuras del encoder)
#define reductora 48
#define refreshRateEncoders 50UL // en milisengudos
//-Controlador-
#define puntoEquilibrio 60  // PWM mínimo que se aplica para estar cercanos al movimiento del robot (POSIBLE IDEA: ptoEq 75 y controlar para que no se mueva (dentro de un rango y durante un tiempo se asume 0))
#define referenciaInicial 0 // referencia inicial de control de velocidad
#define Kp 1
#define Ki 0.001
#define Kd 0
#define SwitchDistance 0.3
#define errorAdmitidoDistance 0.1

/*---------------VARIABLES GLOBALES-------------------*/
int ticksA = 0;
int ticksB = 0;
float wA = 0, wB = 0;
const int ENC1 = pinEncoder1;
const int ENC2 = pinEncoder2;
movimientoCoche motorA(pinENA, pinIN1, pinIN2);
movimientoCoche motorB(pinENB, pinIN3, pinIN4);
struct pose_d
{
  float x_d;
  float y_d;
  float phi_d;
};
struct pose
{
  float x;
  float y;
  float phi;
};
struct goal
{
  float x;
  float y;
};
struct goal posicionObjetivo[] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
struct pose poseRobot = {0, 0, 0};
struct pose_d pose_dRobot;
float R = 0.0325;
float b = 0.1;
/*------------------CABECERAS DE FUNCIONES/SUBRUTINAS  Y VARIABLES GLOBALES ASOCIADAS----------------------------------------*/
void aplicaSignal(movimientoCoche &motor, int signalControl, float RefVelAplicada);
void comprueba_cambio_sentido(float RefVel, float *RefVel_aplicada, movimientoCoche &motor);
void ISR_incrTicksA();
void ISR_incrTicksB();
int sign(float a);
void refresh_interrupt();
void calculaVelAng(void);
void odometria_ModCin();
int eligeControl(float Medida, Controlador &Control1, Controlador &Control2, float referencia, float switchParam);
float calculaErrorAng(float RefAng,float phi);

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
  Timer5.attachInterrupt(refresh_interrupt);
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
  static float RefAng = 0;

  static Controlador ControlVelocidadA(Kp, Ki, Kd, -255 + puntoEquilibrio, 255 - puntoEquilibrio);
  static Controlador ControlVelocidadB(Kp, Ki, Kd, -255 + puntoEquilibrio, 255 - puntoEquilibrio);
  static Controlador Distancia(-1000, -0.005, -Kd, -80, 80);
  static Controlador Orientacion(4, 0.0005, 0, -30, 30);
  // static Controlador VelAngFar(PI/2/90, 0.00005, Kd, -200, 200);
  // static Controlador VelAngFar(PI/2/90, 0.00005, Kd, -200, 200);
  static float deltaVel;
  static int PWM_A, PWM_B;
  static float deltaX = 0, deltaY = 0;
  static int estadoM = 0; // 0=CalculoRefs 1=lineaRecta 2=Giro
  static int indicePunto = 0;
  static float moduloDistancia = 0, errorAng=0;

  
  do
  {
    moduloBluetooth.recibeDatosBluetooth(&RefVelA, &RefVelB, &powerON, ControlVelocidadA, ControlVelocidadB, motorA, motorB); // leer si se ha recibido alguna orden
  } while (!powerON);

  switch (estadoM)
  {
  case 0: // calculoRefs (estado inestable)
    indicePunto++;
    if (indicePunto > 3)
    {
      indicePunto = 0;
    }
    if(indicePunto!=0)
    {
    deltaX = posicionObjetivo[indicePunto].x-posicionObjetivo[indicePunto-1].x;
    deltaY = posicionObjetivo[indicePunto].y-posicionObjetivo[indicePunto-1].y;
    }
    else
    {
     deltaX = posicionObjetivo[indicePunto].x-posicionObjetivo[3].x;
    deltaY = posicionObjetivo[indicePunto].y-posicionObjetivo[3].y;     
    }

    RefAng = (float)atan2(deltaY, deltaX);
    if (RefAng == -PI) // quitar -PI del intervalo para evitar ambiguedades
      RefAng += 2 * PI;
    RefAng *= 180.0 / PI;
    estadoM = 1;
    break;

  case 1: // Giro
    errorAng = calculaErrorAng(RefAng,poseRobot.phi);
    deltaVel = Orientacion.calcularSalida(errorAng);
    RefVelA = deltaVel / 2;
    RefVelB = -deltaVel / 2;
    if (abs(errorAng) < 5 && motorA.sentidoGiro==0 && motorB.sentidoGiro==0)
    {
      estadoM = 2;
            
      Orientacion.resetPID();
      Distancia.resetPID();
    }

    break;

  case 2: // lineaRecta
    deltaX = posicionObjetivo[indicePunto].x - poseRobot.x;
    deltaY = posicionObjetivo[indicePunto].y - poseRobot.y;
    moduloDistancia = sqrt(pow(poseRobot.x - posicionObjetivo[indicePunto].x, 2) + pow(poseRobot.y - posicionObjetivo[indicePunto].y, 2));
    if (moduloDistancia > errorAdmitidoDistance) 
    {
      RefAng = (float)atan2(deltaY, deltaX);
      if (RefAng == -PI) // quitar -PI del intervalo para evitar ambiguedades
        RefAng += 2 * PI;
      RefAng *= 180.0 / PI;
    }

    RefVelA = Distancia.calcularSalida(0-moduloDistancia);
    RefVelB = RefVelA;
    errorAng = calculaErrorAng(RefAng,poseRobot.phi);
    deltaVel = Orientacion.calcularSalida(errorAng);
    RefVelA += deltaVel / 2;
    RefVelB += -deltaVel / 2;
    
    if (moduloDistancia < errorAdmitidoDistance ) // cuando el error sea pequeño y el robot ya no tenga inercia (esté parado)
    {
      estadoM = 0;
      Orientacion.resetPID();
    } 
    break;

  default:
    break;
  }

  // Comprobamos si el signo de la referencia ha cambiado
  comprueba_cambio_sentido(RefVelA, &RefVelA_aplicada, motorA);
  comprueba_cambio_sentido(RefVelB, &RefVelB_aplicada, motorB);

  if(RefVelA_aplicada==0 || RefVelB_aplicada==0) //para conseguir que ambas ruedas empiecen a moverse a la vez
  {
    RefVelA_aplicada=0;
    RefVelB_aplicada=0;
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

  aplicaSignal(motorA, PWM_A, RefVelA_aplicada);
  aplicaSignal(motorB, PWM_B, RefVelB_aplicada);

  moduloBluetooth.DebugValuesBluetooth(RefVelA_aplicada, RefVelB_aplicada, moduloDistancia, RefAng, deltaX, deltaY, poseRobot.x, poseRobot.y, poseRobot.phi, estadoM);
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

void ISR_incrTicksA()
{
  ticksA++;
}

void ISR_incrTicksB()
{
  ticksB++;
}

void refresh_interrupt()
{
  calculaVelAng();
  odometria_ModCin();
  poseRobot.x += pose_dRobot.x_d * 0.05;
  poseRobot.y += pose_dRobot.y_d * 0.05;
  poseRobot.phi += pose_dRobot.phi_d * 0.05 * 180 / PI;
  // cuadrar en rango (-180,180]
  if (poseRobot.phi <= -180)
  {
    poseRobot.phi += 360;
  }
  else if (poseRobot.phi > 180)
  {
    poseRobot.phi -= 360;
  }
}

void calculaVelAng(void)
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
      // Serial.print(ticksA);
      // Serial.println();
      wA = ((float)ticksA * 1000.0 * 60.0) / ((float)npolos * (float)reductora * refreshRateEncoders); // número de vueltas/tiempo entre medición [rps] -> *60 [rpm]
    }
    else
    {
      wA = -((float)ticksA * 1000.0 * 60.0) / ((float)npolos * (float)reductora * refreshRateEncoders);
    }
  }

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

  ticksA = 0;
  ticksB = 0;
}

void odometria_ModCin()
{
  float phi = poseRobot.phi * PI / 180; // paso a radianes
  float wi = wB * 2 * PI / 60;
  float wd = wA * 2 * PI / 60;

  pose_dRobot.x_d = R / 2 * (wi * cos(phi) + wd * cos(phi));
  pose_dRobot.y_d = R / 2 * (wi * sin(phi) + wd * sin(phi));
  pose_dRobot.phi_d = -(R / b) * wi + (R / b) * wd;
}

int eligeControl(float Medida, Controlador &Control1, Controlador &Control2, float referencia, float switchParam)
{
  int res;
  if (abs(Medida - (float)referencia) > switchParam)
  {
    res = Control1.calcularSalida((float)referencia - Medida);
    Control2.resetPID();
  }
  else
  {
    res = Control2.calcularSalida((float)referencia - Medida);
    Control1.resetPID();
  }
  return res;
}

float calculaErrorAng(float RefAng,float phi)
{
    float errorAng;
    if (phi > RefAng)
    {
      if (abs(360 + RefAng - phi) < phi - RefAng)
        errorAng = 360 + RefAng - phi;
      else
        errorAng = RefAng - phi;
    }
    else
    {
      if (abs(360 - RefAng + phi) < RefAng - phi)
        errorAng = -(360 - RefAng + phi);
      else
        errorAng = RefAng - phi;
    }
    return errorAng;
}