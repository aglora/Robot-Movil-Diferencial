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
  ErrorAcumulativo = 0;
  FlagFirstValue = true;
}

int Controlador::calcularSalida(float MeasuredError)
{
  float SalidaControlador;
  double tiempoActual, tiempoTranscurrido;
  float ErrorDerivativo;

  tiempoActual = millis();
  tiempoTranscurrido = tiempoActual - tiempoAnterior;

  if (FlagFirstValue)
  {
    ErrorDerivativo = 0;
    tiempoTranscurrido = 0;
    FlagFirstValue = false;
  }
  else
  {
    ErrorDerivativo = (MeasuredError - ErrorAnterior) / tiempoTranscurrido;
    ErrorAcumulativo += MeasuredError * tiempoTranscurrido;
  }

  SalidaControlador = round(Kp * MeasuredError + Ki * ErrorAcumulativo + Kd * ErrorDerivativo); // calcular la salida del PID
  if (SalidaControlador > max)                                                              // saturar señal de control
  {
    ErrorAcumulativo -= MeasuredError * tiempoTranscurrido; //efecto antiwindup (dejo de acumular error)
    SalidaControlador = max ;
  }
  else if (SalidaControlador < min)
  {
    ErrorAcumulativo -= MeasuredError * tiempoTranscurrido; //efecto antiwindup (dejo de acumular error)
    SalidaControlador = min ;
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

void Bluetooth::DebugValuesBluetooth(float RefVelA, float RefVelB, float moduloDist, float RefAng, float deltax, float deltay, float x, float y, float phi, int estadoM)
{
//  Serial1.print(TiempoTranscurridoEnc);
//  Serial1.print(", ");
  Serial1.print(RefVelA);
  Serial1.print(", ");
  Serial1.print(RefVelB);
  Serial1.print(", ");
  Serial1.print(moduloDist);
  Serial1.print(", ");
  Serial1.print(RefAng);
  Serial1.print(", ");
  Serial1.print(deltax);
  Serial1.print(", ");
  Serial1.print(deltay);
  Serial1.print(", ");
  Serial1.print(x);
  Serial1.print(", ");
  Serial1.print(y);
  Serial1.print(", ");
  Serial1.print(phi);
  Serial1.print(", ");
  Serial1.print(estadoM);
  Serial1.println();

/*  Serial.print(TiempoTranscurridoEnc);
  Serial.print(", ");
  Serial.print(w1);
  Serial.print(", ");
  Serial.print(w2);
  Serial.print(", ");
  Serial.print(Ref_Vel);
  Serial.print(", ");
  Serial.print(modo);
  Serial.print(", ");
  Serial.print(SalidaControlador[0]);
  Serial.print(", ");
  Serial.print(SalidaControlador[1]);
  Serial.println();*/
}

void Bluetooth::recibeDatosBluetooth(float *referenciaA, float *referenciaB, bool *powerON, Controlador &controlA, Controlador &controlB, movimientoCoche &motorA, movimientoCoche &motorB)
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
      controlA.resetPID();
      controlB.resetPID();      
    }
    else if (Buffer.substring(0, 4).equals("PID A "))
    {
      String aux = Buffer.substring(4);
      int n = aux.indexOf(' ');
      float kp = aux.substring(0, n).toFloat();
      aux = aux.substring(n + 1);
      n = aux.indexOf(' ');
      float ki = aux.substring(0, n).toFloat();
      aux = aux.substring(n + 1);
      float kd = aux.toFloat();
      controlA.setGain(kp, ki, kd);
    }
    else if (Buffer.substring(0, 4).equals("PID B "))
    {
      String aux = Buffer.substring(4);
      int n = aux.indexOf(' ');
      float kp = aux.substring(0, n).toFloat();
      aux = aux.substring(n + 1);
      n = aux.indexOf(' ');
      float ki = aux.substring(0, n).toFloat();
      aux = aux.substring(n + 1);
      float kd = aux.toFloat();
      controlB.setGain(kp, ki, kd);
    }
    else if (Buffer.substring(0, 6).equals("REF A "))
    {
      String aux = Buffer.substring(6);
      *referenciaA = aux.toInt();
    }
    else if (Buffer.substring(0, 6).equals("REF B "))
    {
      String aux = Buffer.substring(6);
      *referenciaB = aux.toInt();
    }
    else if (Buffer.substring(0,4).equals("REF "))
    {
      String aux = Buffer.substring(4);
      *referenciaA = aux.toInt();
      *referenciaB = aux.toInt();
    }
  }
}

//---------------------------------------------------