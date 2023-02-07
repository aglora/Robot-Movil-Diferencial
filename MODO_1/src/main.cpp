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
#define EchoPin 13
#define TriggerPin 12
//-Controlador-
#define puntoEquilibrio 65   // PWM mínimo que se aplica para estar cercanos al movimiento del robot
#define referenciaInicial 30 // referencia inicial de control de distancia frontal
#define SwitchDistance 5     // Distancia en la cual cambiamos de controlFar a controlNear
#define Kp_far_inic 1.5
#define Ki_far_inic 0
#define Kd_far_inic 0
#define Kp_near_inic 1
#define Ki_near_inic 0.0001
#define Kd_near_inic 0
#define minimo 0  // expresado en incremento respecto punto de equilibrio
#define maximo 50 // expresado en incremento respecto punto de equilibrio

/*------------------CABECERAS DE FUNCIONES/SUBRUTINAS  Y VARIABLES GLOBALES ASOCIADAS----------------------------------------*/
int eligeControl(float Medida, Controlador &ControlMotoresNear, Controlador &ControlMotoresFar, unsigned int referencia);
int aplicaSignal(movimientoCoche &motor, int signalControl);

/*========================================================================================================================*/

void setup()
{
  // Configuración pines ultrasonidos
  pinMode(TriggerPin, OUTPUT);
  pinMode(EchoPin, INPUT);

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
  Serial.begin(9600);
}

void loop()
{
  static unsigned int Ref_Distancia = referenciaInicial;
  static bool powerON = false;
  static Bluetooth moduloBluetooth;
  static SensorUltrasonidos sonar(TriggerPin, EchoPin); 
  static float distancia;
  static Controlador ControlMotoresFar(Kp_far_inic, Ki_far_inic, Kd_far_inic, minimo, maximo);     
  static Controlador ControlMotoresNear(Kp_near_inic, Ki_near_inic, Kd_near_inic, minimo, maximo); 
  static movimientoCoche motorA(pinENA, pinIN1, pinIN2);
  static movimientoCoche motorB(pinENB, pinIN3, pinIN4);

  do
  {
    moduloBluetooth.recibeDatosBluetooth(&Ref_Distancia, &powerON, ControlMotoresNear, ControlMotoresFar, motorA, motorB); // leer si se ha recibido alguna orden
  } while (!powerON);                                                                                                      // esperar órdenes hasta que el robot se encienda

  // Tomar medida con ultrasonidos
  distancia = sonar.tomaMedidaUltrasonidos();
  // Calcular señal de control
  int PWM = eligeControl(distancia, ControlMotoresNear, ControlMotoresFar, Ref_Distancia);

  // Actuar sobre los motores
  aplicaSignal(motorA, PWM);
  PWM = aplicaSignal(motorB, PWM);

  // Enviar datos de telemetría por Bluetooth
  moduloBluetooth.DebugValuesBluetooth(Ref_Distancia, distancia, PWM);
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
