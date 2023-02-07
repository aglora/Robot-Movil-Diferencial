// Librería para las clases del coche. Creado por Sergio León Doncel (21 Octubre 2022)

#include "ModulesCarDrivers.hpp"

//=================================================================================
#define maxVariation 10       // máxima variabilidad entre medidas de distancia admitida como correcta
#define MaxOutlierValues 5000 // número máximo de medidas pico dadas por no válidas

SensorUltrasonidos::SensorUltrasonidos(unsigned char _TriggerPin, unsigned char _EchoPin)
{
  TriggerPin = _TriggerPin;
  EchoPin = _EchoPin;
}

float SensorUltrasonidos::tomaMedidaUltrasonidos(void)
{
  digitalWrite(TriggerPin, LOW); // para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(4);
  digitalWrite(TriggerPin, HIGH); // generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);
  float duration = pulseIn(EchoPin, HIGH);               // medimos el tiempo entre pulsos, en microsegundos
  float MedidaActual = (float)(duration * 10 / 292 / 2); // convertimos a distancia, en cm
  return MedidaActual;
}

//------------------------------------------------------------------------

Controlador::Controlador(float _Kp, float _Ki, float _Kd, int _min, int _max)
{
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;

  min = _min;
  max = _max;

  resetPID();
}

void Controlador::setGain(float Kp, float Ki, float Kd)
{
  Kp = Kp;
  Ki = Ki;
  Kd = Kd;
}

void Controlador::resetPID(void)
{
  tiempoAnterior = 0;
  ErrorAnterior = 0;
  ErrorAcumulativo = 0;
  FlagFirstValue = true;
}

int Controlador::calcularSalida(float MeasuredError)
{
  int SalidaControlador;
  double tiempoActual, tiempoTranscurrido;
  float ErrorDerivativo;

  tiempoActual = millis();
  tiempoTranscurrido = tiempoActual - tiempoAnterior;

  if (FlagFirstValue)
  {
    ErrorAcumulativo = 0;
    ErrorDerivativo = 0;
    tiempoTranscurrido = 0;
    FlagFirstValue = false;
  }
  else
  {
    ErrorDerivativo = (MeasuredError - ErrorAnterior) / tiempoTranscurrido;
    ErrorAcumulativo += MeasuredError * tiempoTranscurrido;
  }

  SalidaControlador = -round(Kp * MeasuredError + Ki * ErrorAcumulativo + Kd * ErrorDerivativo); // calcular la salida del PID
  if (SalidaControlador > max)                                                              // saturar señal de control
  {
    SalidaControlador = max;
    ErrorAcumulativo -= MeasuredError * tiempoTranscurrido;
  }
  else if (SalidaControlador < min)
  {
    SalidaControlador = min;
    ErrorAcumulativo -= MeasuredError * tiempoTranscurrido;
  }
  ErrorAnterior = MeasuredError;
  tiempoAnterior = tiempoActual;
  return SalidaControlador;
}

//----------------------------------------------------------------------------

movimientoCoche::movimientoCoche(unsigned char _pinEnable, unsigned char _pinBackward, unsigned char _pinForward)
{
  pinEnable = _pinEnable;
  pinBackward = _pinBackward;
  pinForward = _pinForward;
}

movimientoCoche::~movimientoCoche() {}

void movimientoCoche::forward(unsigned char speed) const
{
  // Configurar giro hacia delante
  digitalWrite(pinForward, HIGH);
  digitalWrite(pinBackward, LOW);

  // Señales PWM
  analogWrite(pinEnable, speed);
}

void movimientoCoche::backward(unsigned char speed) const
{
  // Configurar giro hacia atrás
  digitalWrite(pinForward, LOW);
  digitalWrite(pinBackward, HIGH);

  // Señales PWM
  analogWrite(pinEnable, speed);
}

void movimientoCoche::stop(void) const
{
  digitalWrite(pinForward, LOW);
  digitalWrite(pinBackward, LOW);

  digitalWrite(pinEnable, 0);
}

//-------------------------------------------------------------------------------

void Bluetooth::DebugValuesBluetooth(float d1, float d2, float Ref_Distancia, float distancia, float referencia_ang, float theta, unsigned char PWM_A, unsigned char PWM_B)
{
  Serial1.print(d1); // Enviamos serialmente el valor de la distancia

  Serial1.print(", ");
  Serial1.print(d2); // Enviamos serialmente el valor de la distancia

  Serial1.print(", ");
  Serial1.print(Ref_Distancia); 

  Serial1.print(", ");
  Serial1.print(distancia); 

  Serial1.print(", ");
  Serial1.print(referencia_ang);

  Serial1.print(", ");
  Serial1.print(theta);  

  Serial1.print(", ");
  Serial1.print(PWM_A);

  Serial1.print(", ");
  Serial1.print(PWM_B);

  Serial1.println();
}

void Bluetooth::recibeDatosBluetooth(float *referencia, bool *powerON, Controlador &controlOrientacion, movimientoCoche &motorA, movimientoCoche &motorB)
{
  if (Serial1.available())
  {                                                // leer referencia de distancia por Bluetooth
    String Buffer = Serial1.readStringUntil('\n'); // variable auxiliar para guardar datos recibidos temporalmente hasta su procesamiento
    Buffer.trim();                                 // elimina de la cadena de texto caracteres especiales (LF,CR...)
    if (Buffer.equals("ON"))
    {
      *powerON = true;
    }
    else if (Buffer.equals("OFF"))
    {
      *powerON = false;
      motorA.stop();
      motorB.stop();
    }
    else if (Buffer.substring(0, 6).equals("PID_O "))
    {
      String aux = Buffer.substring(6);
      int n = aux.indexOf(' ');
      float kp = aux.substring(0, n).toFloat();
      aux = aux.substring(n + 1);
      n = aux.indexOf(' ');
      float ki = aux.substring(0, n).toFloat();
      aux = aux.substring(n + 1);
      float kd = aux.toFloat();
      controlOrientacion.setGain(kp, ki, kd);
    }
    else if (Buffer.substring(0, 4).equals("REF "))
    {
      String aux = Buffer.substring(4);
      *referencia = aux.toInt();
    }
  }
}

//---------------------------------------------------