#ifndef PIDCONTROLLERESP32_H
#define PIDCONTROLLERESP32_H

#include <Arduino.h>

enum PIDType {
  PID_BASIC,
  PID_IIR,
  PID_FILTERED_D
};

class PIDControllerESP32 {
public:
  PIDControllerESP32(float Kp, float Ki, float Kd, float dt, PIDType mode);

  void setSetpoint(float sp);
  void updateMeasurement(float mv);
  float compute();
  void setTunings(float Kp, float Ki, float Kd);

private:
  float Kp, Ki, Kd;
  float dt;
  PIDType mode;

  float setpoint;
  float measured_value;

  // PID básico
  float previous_error;
  float integral;

  // PID IIR y filtrado
  float error[3];
  float d[2];
  float fd[2];

  float A0, A1, A2; // coeficientes IIR

  // Filtro pasa-bajo
  float alpha_1, alpha_2;

  void updateErrorHistory(float e);
  void computeIIRCoefficients();
  void computeFilterConstants();
};

#endif
