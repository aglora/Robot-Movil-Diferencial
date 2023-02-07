#include <Arduino.h>
#include "ModulesCarDrivers.hpp"

/*-----------------DEFINICIONES PARA PARÁMETROS----------------------*/
#define MODO 4
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
//-Controlador-
#define puntoEquilibrio 45   // PWM mínimo que se aplica para estar cercanos al movimiento del robot
#define referenciaInicial 50 // referencia inicial de control de distancia frontal
#define SwitchDistance 5     // Distancia en la cual cambiamos de controlFar a controlNear
#define Kp_ditancia_inic 1
#define Ki_distancia_inic 0.00005
#define Kd_distancia_inic 0
#define Kp_orientacion_inic 1
#define Ki_orientacion_inic 0.0005
#define Kd_orientacion_inic 0

/*------------------CABECERAS DE FUNCIONES/SUBRUTINAS  Y VARIABLES GLOBALES ASOCIADAS----------------------------------------*/
int aplicaSignal(movimientoCoche &motor, int signalControl);

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

  // Configuración puerto bluetooth para telemetría
  Serial1.begin(9600);

  // Configuración puerto usb para depuración
  // Serial.begin(9600);
}

void loop()
{
  static float Ref_Distancia = referenciaInicial;
  static bool powerON = false;
  static Bluetooth moduloBluetooth;
  static SensorUltrasonidos sonarA(TriggerPinA, EchoPinA); // instancia del objeto sonar de clase Ultrasonidos
  static SensorUltrasonidos sonarB(TriggerPinB, EchoPinB); // instancia del objeto sonar de clase Ultrasonidos
  static float distancia;
  static float d1, d2;
  static float referencia_ang;
  static float theta;
  const int delta = 14;
  static Controlador ControlDistancia(Kp_ditancia_inic, Ki_distancia_inic, Kd_distancia_inic,-10,10);
  static Controlador ControlOrientacion(Kp_orientacion_inic, Ki_orientacion_inic, Kd_orientacion_inic, -40, 40);
  static int PWM_A, PWM_B;
  static movimientoCoche motorA(pinENA, pinIN1, pinIN2);
  static movimientoCoche motorB(pinENB, pinIN3, pinIN4);

  do
  {
    moduloBluetooth.recibeDatosBluetooth(&Ref_Distancia, &powerON, ControlOrientacion, motorA, motorB); // leer si se ha recibido alguna orden
  } while (!powerON);                                                                                                                              // esperar órdenes hasta que el robot se encienda

  // Tomar medida con ultrasonidos
  d1 = sonarA.tomaMedidaUltrasonidos();
  d2 = sonarB.tomaMedidaUltrasonidos();

  if (d2 > d1)
  {
    theta = PI / 2 - asin(delta / sqrt(pow(delta, 2) + pow(d2 - d1, 2)));
    distancia = (d2 * cos(theta) - (float)delta / 2.0 * sin(theta) +  d1 * cos(theta) + (float)delta / 2.0 * sin(theta))/2;
  }
  else if (d1 > d2)
  {
    theta = -(PI / 2 - asin(delta / sqrt(pow(delta, 2) + pow(d1 - d2, 2))));
    distancia = (d1 * cos(theta) - (float)delta / 2.0 * sin(theta) + d2 * cos(theta) + (float)delta / 2.0 * sin(theta))/2;
  }
  else
  {
    theta = 0;
    distancia = d1;
  }

  theta = theta * 180 / PI; // cambio de rad a grados

  // Calcular señal de control
  static const int PWM = 25;
  referencia_ang = ControlDistancia.calcularSalida(Ref_Distancia-distancia);
  int delta_PWM = ControlOrientacion.calcularSalida(referencia_ang - theta);

  // Actuar sobre los motores
  if ((delta_PWM > 0 && PWM >= 0) || (delta_PWM < 0 && PWM < 0))
  {
    PWM_A = aplicaSignal(motorA, PWM);
    PWM_B = aplicaSignal(motorB, PWM + delta_PWM);
  }
  else
  {
    PWM_A = aplicaSignal(motorA, PWM - delta_PWM);
    PWM_B = aplicaSignal(motorB, PWM);
  }

  // Enviar datos de telemetría por Bluetooth
  moduloBluetooth.DebugValuesBluetooth(d1, d2, Ref_Distancia, distancia, referencia_ang, theta, PWM_A, PWM_B);
  delay(60); // tiempo entre medidas (60 max según datasheet)
}

int aplicaSignal(movimientoCoche &motor, int signalControl)
{
  int PWM;
  if (signalControl > 0)
  {
    PWM = signalControl + puntoEquilibrio;
    motor.forward(PWM);
  }
  else if (signalControl < 0)
  {
    PWM = abs(signalControl) + puntoEquilibrio;
    motor.backward(PWM);
  }
  else
  {
    PWM = 0;
    motor.stop();
  }
  return PWM;
}
