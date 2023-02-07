#ifndef _MODULESCARDRIVERS_HPP_
//-----------------------------------------------------------------------------------------------------------
#define _MODULESCARDRIVERS_HPP_
#include <Arduino.h>

// Clase asociadas a los sensores ultrasonidos del coche
class SensorUltrasonidos
{
private: // Atributos
  unsigned char TriggerPin, EchoPin;

public:                                                                  // Métodos
  SensorUltrasonidos(unsigned char _TriggerPin, unsigned char _EchoPin); // Constructor
  float tomaMedidaUltrasonidos(void);                                    // Subrutina de medición de distancia + filtro Outlier
};

// Clase asociada al control del coche
class Controlador
{
private: // Atributos
  float Kp, Ki, Kd;
  int min, max;
  double tiempoAnterior;
  float ErrorAnterior, ErrorAcumulativo;
  bool FlagFirstValue;

public:                                                             // Métodos
  Controlador(float _Kp, float _Ki, float _Kd, int _min, int _max); // Constructor
  void setGain(float _Kp, float _Ki, float _Kd);                    // Set
  void resetPID(void);
  int calcularSalida(float MeasuredError);
};

// Clase asociada al movimiento del coche
class movimientoCoche
{
private:
  unsigned char pinEnable;
  unsigned char pinForward;
  unsigned char pinBackward;

public:
  movimientoCoche(unsigned char _pinEnable, unsigned char _pinBackward, unsigned char _pinForward);
  ~movimientoCoche();
  void forward(unsigned char speed) const;
  void backward(unsigned char speed) const;
  void stop(void) const;
};

// Clase asociada al envío de datos y recibo de órdenes por Bluetooth
class Bluetooth
{
public:
  void DebugValuesBluetooth(float d1, float d2, float Ref_Distancia, float distancia, float referencia_ang, float theta, unsigned char PWM_A, unsigned char PWM_B);
  void recibeDatosBluetooth(float *referencia, bool *powerON, Controlador &controlOrientacion, movimientoCoche &motorA, movimientoCoche &motorB);
};

//-----------------------------------------------------------------------------------------------------------
#endif