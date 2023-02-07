#include "Arduino.h"
#include "ModulesCarDrivers.hpp"
#include "TimerFive.h"
#include "Servo.h"

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
#define EchoPinA 18
#define TriggerPinA 12
#define EchoPinB 19
#define TriggerPinB 15
#define EchoPinC 3
#define TriggerPinC 14
//-Encoders-
#define pinEncoder1 20
#define pinEncoder2 21
#define npolos 8 // Número de cambios de tensión correspondientes a una vuelta (doble de ranuras del encoder)
#define reductora 48
#define refreshRateEncoders 50UL // en milisengudos
// Servomotor
#define pinServo 9
//-Controlador-
#define puntoEquilibrio 60  // PWM mínimo que se aplica para estar cercanos al movimiento del robot (POSIBLE IDEA: ptoEq 75 y controlar para que no se mueva (dentro de un rango y durante un tiempo se asume 0))
#define referenciaInicial 0 // referencia inicial de control de velocidad
#define SwitchDistance 0.3
#define errorAdmitidoDistance 0.05
// Evitación Obstaculos
#define distancia_prudente 25 // distancia de separacion de seguridad al obstaculo
#define paso_angular 3        // resolucion angular para barrido del servo
#define umbral_inc_dist 30    // umbral de cambio brusco de distancia para conocer donde acaba obstaculo
#define umbral_ang_recta 1    // umbral para deteccion reincorporacion a punto objetivo inicial

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
struct goal posicionObjetivo = {0, 0};

// Estados
enum estados
{
  espera_goal,            // Estado inicial de espera a recepcción de nueva posicion objetivo
  giro_inicial,           // giro inicial para colocarse mirando hacia donde nos vamos a mover
  inicializacion,         // Calcular ángulo y trayectoria para ir al pto. objetivo en línea recta
  control_pos_objetivo,   // Control del robot para posición objetivo
  barrido_servo_D,        // Barrido del servo hacia derecha para detección de obstaculos
  barrido_servo_I,        // Barrido del servo hacia izquierda para detección de obstaculos
  decision,               // Barrido en ángulo y/o distancias y decidir a qué lado girar
  giro_evitacion,         // Giro tras decision
  avance_busca_obstaculo, // Avance hasta encontrar de nuevo el obstaculo tras bordear una esquina
  avance_evitacion,       // Avance tras giro dado decicido
  avance_prudente         // Avance de una distancia prudente extra para asegurar evitar obstaculo
};
estados estadoM = espera_goal;

struct pose poseRobot = {0, 0, 0};
struct pose_d pose_dRobot;
float R = 0.0325;
float b = 0.1;

bool flagCompruebaObstaculo = false;
int ang_barrido_D = 0;
float medida_ant = 0;
float RefVelA = referenciaInicial;
float RefVelB = referenciaInicial;
Controlador Orientacion(1.5, 0.0005, 0, -30, 30);

Servo OjoFrontal;
float DistanciaFrontal, DistanciaLatD, DistanciaLatI;
SensorUltrasonidos sonicFrontal(TriggerPinC, EchoPinC);
SensorUltrasonidos sonicLatI(TriggerPinB, EchoPinB);
SensorUltrasonidos sonicLatD(TriggerPinA, EchoPinA);
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
float calculaErrorAng(float RefAng, float phi);
void medicionUltrasonidos();
void ISR_recogeEchoD();
void ISR_recogeEchoI();
void ISR_recogeEchoF();
float calcula_RefAngAct();
/*========================================================================================================================*/

void setup()
{
  // Configuración pines ultrasonidos
  pinMode(TriggerPinA, OUTPUT);
  pinMode(EchoPinA, INPUT);
  pinMode(TriggerPinB, OUTPUT);
  pinMode(EchoPinB, INPUT);
  pinMode(TriggerPinC, OUTPUT);
  pinMode(EchoPinC, INPUT);
  attachInterrupt(digitalPinToInterrupt(EchoPinA), ISR_recogeEchoD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EchoPinB), ISR_recogeEchoI, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EchoPinC), ISR_recogeEchoF, CHANGE);

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

  // Configuración servo
  OjoFrontal.attach(pinServo);
  OjoFrontal.write(90);

  // Configuración puerto bluetooth para telemetría
  Serial2.begin(9600);

  // Configuración puerto usb para depuración
  Serial.begin(9600);
}

void loop()
{
  static bool powerON = false;
  static Bluetooth moduloBluetooth;
  static float RefVelA_aplicada = referenciaInicial;
  static float RefVelB_aplicada = referenciaInicial;
  static float RefAng = 0;

  static Controlador ControlVelocidadA(1, 0.001, 0, -255 + puntoEquilibrio, 255 - puntoEquilibrio);
  static Controlador ControlVelocidadB(1, 0.001, 0, -255 + puntoEquilibrio, 255 - puntoEquilibrio);
  static Controlador Distancia(-1000, -0.005, 0, -80, 80);
  static float deltaVel;
  static int PWM_A, PWM_B;
  static float moduloDistancia = 0, errorAng = 0;

  static int ang_barrido_I = 0;
  static bool flagGiro = false; // true = giroD ; false = giroIzq;
  static struct pose pose_corner;
  static float distanciaRecorrida;
  static float RefAngIni, RefAngAct;
  static bool flag_ang_recta = false;
  static float tiempoAnt = millis();
  static float tiempo = millis();

  do
  {
    moduloBluetooth.recibeDatosBluetooth(&RefVelA, &RefVelB, &powerON, ControlVelocidadA, ControlVelocidadB, motorA, motorB, 0, 0); // leer si se ha recibido alguna orden
  } while (!powerON);

  // medicionUltrasonidos();

  switch (estadoM)
  {

  case espera_goal:
    posicionObjetivo.x = 0;
    posicionObjetivo.y = 0;
    // Colocamos SR global en la poscion actual del robot (Resetamos poseRobot)
    poseRobot = {0, 0, 0};
    while (posicionObjetivo.x == 0 && posicionObjetivo.y == 0)
    {
      moduloBluetooth.recibeDatosBluetooth(&RefVelA, &RefVelB, &powerON, ControlVelocidadA, ControlVelocidadB, motorA, motorB, &(posicionObjetivo.x), &(posicionObjetivo.y)); // leer si se ha recibido alguna orden
    }
    estadoM = inicializacion;
    break;

  case inicializacion:
    OjoFrontal.write(90);
    RefAngIni = calcula_RefAngAct();
    estadoM = giro_inicial;
    break;

  case giro_inicial:
    errorAng = calculaErrorAng(RefAngIni, poseRobot.phi);
    deltaVel = Orientacion.calcularSalida(errorAng);
    RefVelA = deltaVel / 2;
    RefVelB = -deltaVel / 2;
    if (abs(errorAng) < 5 && motorA.sentidoGiro == 0 && motorB.sentidoGiro == 0)
    {
      estadoM = control_pos_objetivo;
      flagCompruebaObstaculo = true;
      Orientacion.resetPID();
      Distancia.resetPID();
    }
    break;

  case control_pos_objetivo:
    RefAng = calcula_RefAngAct();
    moduloDistancia = sqrt(pow(poseRobot.x - posicionObjetivo.x, 2) + pow(poseRobot.y - posicionObjetivo.y, 2));

    RefVelA = Distancia.calcularSalida(0 - moduloDistancia);
    RefVelB = RefVelA;
    errorAng = calculaErrorAng(RefAng, poseRobot.phi);
    deltaVel = Orientacion.calcularSalida(errorAng);
    RefVelA += deltaVel / 2;
    RefVelB += -deltaVel / 2;

    if (moduloDistancia < errorAdmitidoDistance) // cuando el error sea pequeño y el robot ya no tenga inercia (esté parado)
    {
      estadoM = espera_goal;
      Orientacion.resetPID();
      RefVelA = 0;
      RefVelB = 0;
    }

    break;

  case barrido_servo_D:
    if ((abs(sonicFrontal.valorMedida - medida_ant) > umbral_inc_dist) || (ang_barrido_D >= 90))
    {
      // Inicializacion para barrer angulo I
      OjoFrontal.write(90);
      medida_ant = distancia_prudente;
      ang_barrido_I = 0;
      delay(300);
      estadoM = barrido_servo_I;
    }
    else
    {
      ang_barrido_D += paso_angular;
      OjoFrontal.write(90 - ang_barrido_D);
      medida_ant = sonicFrontal.valorMedida;
    }
    delay(300);
    break;

  case barrido_servo_I:
    if ((abs(sonicFrontal.valorMedida - medida_ant) > umbral_inc_dist) || (ang_barrido_I >= 90))
    {
      OjoFrontal.write(90);
      estadoM = decision;
    }
    else
    {
      ang_barrido_I += paso_angular;
      OjoFrontal.write(90 + ang_barrido_I);
      medida_ant = sonicFrontal.valorMedida;
    }
    delay(300);
    break;

  case decision:
    if (ang_barrido_D <= ang_barrido_I)
    {
      RefAng = poseRobot.phi - 90; // RefAng=PosePhi - 90 (respecto a pose actual giro 90 grados a la derecha)
      if (RefAng <= -180)
        RefAng = 360 + RefAng;
      flagGiro = true;
    }
    else
    {
      RefAng = poseRobot.phi + 90; // RefAng=PosePhi + 90 (respecto a pose actual giro 90 grados a la izquierda)
      if (RefAng > 180)
        RefAng = RefAng - 360;
      flagGiro = false;
    }
    estadoM = giro_evitacion;
    break;

  case giro_evitacion:
    errorAng = calculaErrorAng(RefAng, poseRobot.phi);
    deltaVel = Orientacion.calcularSalida(errorAng);
    RefVelA = deltaVel / 2;
    RefVelB = -deltaVel / 2;
    if (abs(errorAng) < 5 && motorA.sentidoGiro == 0 && motorB.sentidoGiro == 0)
    {
      estadoM = avance_busca_obstaculo;
      RefVelA = 0;
      RefVelB = 0;
      Orientacion.resetPID();
      Distancia.resetPID();
      flagCompruebaObstaculo = true;
    }
    break;

  case avance_busca_obstaculo: // La primera vez es inestable (sirve para cuando bordeamos esquinas)
    if (((sonicLatD.valorMedida < distancia_prudente + umbral_inc_dist) && (!flagGiro)) || ((sonicLatI.valorMedida < distancia_prudente + umbral_inc_dist) && (flagGiro)))
    {
      estadoM = avance_evitacion;
      // No reseteamos controladores usados porque se sigue avanzando en el mismo sentido
    }
    else
    {
      RefVelA = 50;
      RefVelB = RefVelA;
      errorAng = calculaErrorAng(RefAng, poseRobot.phi);
      deltaVel = Orientacion.calcularSalida(errorAng);
      RefVelA += deltaVel / 2;
      RefVelB += -deltaVel / 2;
    }
    // Comprobación salida hacia punto objetivo
    RefAngAct = calcula_RefAngAct();
    if ((abs(RefAngAct - RefAngIni) <= umbral_ang_recta))
    {
      if (flag_ang_recta)
      {
        estadoM = giro_inicial;
        Orientacion.resetPID();
      }
    }
    break;

  case avance_evitacion:
    RefVelA = 50;
    RefVelB = RefVelA;
    errorAng = calculaErrorAng(RefAng, poseRobot.phi);
    deltaVel = Orientacion.calcularSalida(errorAng);
    RefVelA += deltaVel / 2;
    RefVelB += -deltaVel / 2;

    // Comprobación salida hacia punto objetivo
    RefAngAct = calcula_RefAngAct();
    if (abs(RefAngAct - RefAngIni) <= umbral_ang_recta)
    {
      if (flag_ang_recta)
      {
        estadoM = giro_inicial;
        Orientacion.resetPID();
      }
    }

    else
    {
      flag_ang_recta = true;
      if (((sonicLatD.valorMedida > distancia_prudente + umbral_inc_dist) && (!flagGiro)) || ((sonicLatI.valorMedida > distancia_prudente + umbral_inc_dist) && (flagGiro)))
      {
        pose_corner = poseRobot;
        estadoM = avance_prudente;
        // No reseteamos controladores usados porque se sigue avanzando en el mismo sentido
      }
    }
    break;

  case avance_prudente:
    distanciaRecorrida = sqrt(pow(poseRobot.x - pose_corner.x, 2) + pow(poseRobot.y - pose_corner.y, 2));
    RefVelA = Distancia.calcularSalida(distanciaRecorrida - ((0.5 * distancia_prudente) * 0.01));
    RefVelB = RefVelA;
    errorAng = calculaErrorAng(RefAng, poseRobot.phi);
    deltaVel = Orientacion.calcularSalida(errorAng);
    RefVelA += deltaVel / 2;
    RefVelB += -deltaVel / 2;

    // Comprobación salida hacia punto objetivo
    RefAngAct = calcula_RefAngAct();
    if (abs(RefAngAct - RefAngIni) <= umbral_ang_recta)
    {
      if (flag_ang_recta)
      {
        estadoM = giro_inicial;
        Orientacion.resetPID();
      }
    }
    else
    {
      if (distanciaRecorrida >= ((0.5 * distancia_prudente) * 0.01))
      {
        estadoM = giro_evitacion;
        Orientacion.resetPID();
        Distancia.resetPID();
        RefVelA = 0;
        RefVelB = 0;
        if (flagGiro)
        {
          RefAng = poseRobot.phi + 90;
          if (RefAng > 180)
            RefAng = RefAng - 360;
        }
        else
        {
          RefAng = poseRobot.phi - 90;
          if (RefAng <= -180)
            RefAng = 360 + RefAng;
        }
      }
    }

    break;

  default:
    break;
  }

  // Comprobamos si el signo de la referencia ha cambiado
  comprueba_cambio_sentido(RefVelA, &RefVelA_aplicada, motorA);
  comprueba_cambio_sentido(RefVelB, &RefVelB_aplicada, motorB);

  if (RefVelA_aplicada == 0 || RefVelB_aplicada == 0) // para conseguir que ambas ruedas empiecen a moverse a la vez
  {
    RefVelA_aplicada = 0;
    RefVelB_aplicada = 0;
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

  tiempo = millis() - tiempoAnt;
  tiempoAnt = millis();

  moduloBluetooth.DebugValuesBluetooth(tiempo, RefVelA_aplicada, RefVelB_aplicada, RefAng, posicionObjetivo.x, posicionObjetivo.y, poseRobot.x, poseRobot.y, poseRobot.phi, estadoM, sonicLatD.valorMedida, sonicLatI.valorMedida, sonicFrontal.valorMedida);
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
  static bool flagMedidasUltrasonidos = true;
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

  if (flagMedidasUltrasonidos)
  {
    medicionUltrasonidos();
    flagMedidasUltrasonidos = false;
  }
  else
    flagMedidasUltrasonidos = true;
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

float calculaErrorAng(float RefAng, float phi)
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

void medicionUltrasonidos()
{
  sonicFrontal.tomaMedidaUltrasonidos();
  sonicLatD.tomaMedidaUltrasonidos();
  sonicLatI.tomaMedidaUltrasonidos();
}

void ISR_recogeEchoD()
{
  static bool flagRising = true;
  static unsigned long tiempoRising = 0;

  if (flagRising)
  {
    tiempoRising = micros();
    flagRising = false;
  }
  else
  {
    sonicLatD.valorMedida = (float)((micros() - tiempoRising) / 58.0); // convertimos a distancia, en cm
    flagRising = true;
  }
}

void ISR_recogeEchoI()
{
  static bool flagRising = true;
  static unsigned long tiempoRising = 0;

  if (flagRising)
  {
    tiempoRising = micros();
    flagRising = false;
  }
  else
  {
    sonicLatI.valorMedida = (float)((micros() - tiempoRising) / 58.0); // convertimos a distancia, en cm
    flagRising = true;
  }
}

void ISR_recogeEchoF()
{
  static bool flagRising = true;
  static unsigned long tiempoRising = 0;

  if (flagRising)
  {
    tiempoRising = micros();
    flagRising = false;
  }
  else
  {
    sonicFrontal.valorMedida = (float)((micros() - tiempoRising) / 58.0); // convertimos a distancia, en cm
    flagRising = true;
  }

  if ((sonicFrontal.valorMedida <= distancia_prudente) && flagCompruebaObstaculo)
  {
    Orientacion.resetPID();
    RefVelA = 0;
    RefVelB = 0;
    // Inicializacion para barrer angulo D
    OjoFrontal.write(90);
    medida_ant = distancia_prudente;
    ang_barrido_D = 0;
    estadoM = barrido_servo_D;
    flagCompruebaObstaculo = false;
  }
}

float calcula_RefAngAct()
{
  float deltaX;
  deltaX = posicionObjetivo.x - poseRobot.x;
  float deltaY;
  deltaY = posicionObjetivo.y - poseRobot.y;
  float RefAng;
  RefAng = (float)atan2(deltaY, deltaX);
  if (RefAng == -PI)
    RefAng += 2 * PI;
  RefAng *= 180.0 / PI;
  return RefAng;
}
