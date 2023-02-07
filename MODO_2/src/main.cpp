#include <Arduino.h>
#include "ModulesCarDrivers.hpp"

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
//-Controlador-
#define puntoEquilibrio 65   // PWM mínimo que se aplica para estar cercanos al movimiento del robot
#define referenciaInicial 30 // referencia inicial de control de distancia frontal
#define SwitchDistance 5     // Distancia en la cual cambiamos de controlFar a controlNear
#define Kp_far_inic 1.5      // 1
#define Ki_far_inic 0
#define Kd_far_inic 0
#define Kp_near_inic 3
#define Ki_near_inic 0.0001
#define Kd_near_inic 0
#define Kp_orientacion_inic 1
#define Ki_orientacion_inic 0.001
#define Kd_orientacion_inic 0
#define minimo 0  // expresado en incremento respecto punto de equilibrio
#define maximo 50 // expresado en incremento respecto punto de equilibrio

/*------------------CABECERAS DE FUNCIONES/SUBRUTINAS  Y VARIABLES GLOBALES ASOCIADAS----------------------------------------*/
int eligeControl(float Medida, Controlador &ControlMotoresNear, Controlador &ControlMotoresFar, unsigned int referencia);
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
  static unsigned int Ref_Distancia = referenciaInicial;
  static bool powerON = false;
  static Bluetooth moduloBluetooth;
  static SensorUltrasonidos sonarA(TriggerPinA, EchoPinA); // instancia del objeto sonar de clase Ultrasonidos
  static SensorUltrasonidos sonarB(TriggerPinB, EchoPinB); // instancia del objeto sonar de clase Ultrasonidos
  static float d1, d2;
  static float theta, distancia;
  const int delta = 12;
  static Controlador ControlDistanciaFar(Kp_far_inic, Ki_far_inic, Kd_far_inic, minimo, maximo);     // instancia del objeto ControlMotoresFar de clase Controlador
  static Controlador ControlDistanciaNear(Kp_near_inic, Ki_near_inic, Kd_near_inic, minimo, maximo); // instancia del objeto ControlMotoresNear de clase Controlador
  static Controlador ControlOrientacion(Kp_orientacion_inic, Ki_orientacion_inic, Kd_orientacion_inic, -10, 10);
  static int PWM_A, PWM_B;
  static movimientoCoche motorA(pinENA, pinIN1, pinIN2);
  static movimientoCoche motorB(pinENB, pinIN3, pinIN4);

  do
  {
    moduloBluetooth.recibeDatosBluetooth(&Ref_Distancia, &powerON, ControlDistanciaNear, ControlDistanciaFar, ControlOrientacion, motorA, motorB); // leer si se ha recibido alguna orden
  } while (!powerON);                                                                                                                              // esperar órdenes hasta que el robot se encienda

  // Tomar medida con ultrasonidos
  d1 = sonarA.tomaMedidaUltrasonidos();
  d2 = sonarB.tomaMedidaUltrasonidos();

  if (d2 > d1)
  {
    theta = PI / 2 - asin(delta / sqrt(pow(delta, 2) + pow(d2 - d1, 2)));
    distancia = d2 * cos(theta) - delta / 2 * sin(theta);
  }
  else if (d1 > d2)
  {
    theta = -(PI / 2 - asin(delta / sqrt(pow(delta, 2) + pow(d1 - d2, 2))));
    distancia = d1 * cos(theta) - delta / 2 * sin(theta);
  }
  else
    theta = 0;
  theta = theta * 180 / PI; // cambio de rad a grados

  // Calcular señal de control
  int PWM = eligeControl(distancia, ControlDistanciaNear, ControlDistanciaFar, Ref_Distancia);
  int delta_PWM = ControlOrientacion.calcularSalida(0 - theta);

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
  moduloBluetooth.DebugValuesBluetooth(Ref_Distancia, d1, d2, distancia, theta, PWM_A, PWM_B);
  delay(60); // tiempo entre medidas (60 max según datasheet)
}

int eligeControl(float Medida, Controlador &ControlMotoresNear, Controlador &ControlMotoresFar, unsigned int referencia)
{
  int PWM_calculado;
  if (abs(Medida - (float)referencia) > SwitchDistance)
  {
    PWM_calculado = ControlMotoresFar.calcularSalida((float)referencia - Medida);
    ControlMotoresNear.resetPID();
  }
  else
  {
    PWM_calculado = ControlMotoresNear.calcularSalida((float)referencia - Medida);
    ControlMotoresFar.resetPID();
  }
  return PWM_calculado;
}

int aplicaSignal(movimientoCoche &motor, int signalControl)
{
  int PWM;
  if (signalControl > 2)
  {
    PWM = signalControl + puntoEquilibrio;
    motor.forward(PWM);
  }
  else if (signalControl < -2)
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
