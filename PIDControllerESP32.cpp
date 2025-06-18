#include "PIDControllerESP32.h"

PIDControllerESP32::PIDControllerESP32(float Kp_, float Ki_, float Kd_, float dt_, PIDType mode_)
  : Kp(Kp_), Ki(Ki_), Kd(Kd_), dt(dt_), mode(mode_), setpoint(0), measured_value(0),
    previous_error(0), integral(0)
{
  for (int i = 0; i < 3; i++) error[i] = 0;
  for (int i = 0; i < 2; i++) d[i] = fd[i] = 0;

  if (mode == PID_IIR) computeIIRCoefficients();
  if (mode == PID_FILTERED_D) computeFilterConstants();
}

void PIDControllerESP32::setSetpoint(float sp) {
  setpoint = sp;
}

void PIDControllerESP32::updateMeasurement(float mv) {
  measured_value = mv;
}

void PIDControllerESP32::updateErrorHistory(float e) {
  error[2] = error[1];
  error[1] = error[0];
  error[0] = e;
}

void PIDControllerESP32::setTunings(float Kp_, float Ki_, float Kd_) {
  Kp = Kp_; Ki = Ki_; Kd = Kd_;
  if (mode == PID_IIR) computeIIRCoefficients();
  if (mode == PID_FILTERED_D) computeFilterConstants();
}

void PIDControllerESP32::computeIIRCoefficients() {
  A0 = Kp + Ki * dt + Kd / dt;
  A1 = -Kp - 2 * Kd / dt;
  A2 = Kd / dt;
}

void PIDControllerESP32::computeFilterConstants() {
  float N = 5.0; // constante de atenuación
  float tau = Kd / (Kp * N);
  float alpha = dt / (2 * tau);
  alpha_1 = alpha / (alpha + 1);
  alpha_2 = (alpha - 1) / (alpha + 1);
}

float PIDControllerESP32::compute() {
  float e = setpoint - measured_value;
  updateErrorHistory(e);

  float output = 0;

  switch (mode) {
    case PID_BASIC: {
      integral += e * dt;
      float derivative = (e - previous_error) / dt;
      output = Kp * e + Ki * integral + Kd * derivative;
      previous_error = e;
      break;
    }

    case PID_IIR: {
      output = A0 * error[0] + A1 * error[1] + A2 * error[2];
      break;
    }

    case PID_FILTERED_D: {
      output += (Kp + Ki * dt) * error[0] - Kp * error[1];

      d[1] = d[0];
      d[0] = (Kd / dt) * (error[0] - 2 * error[1] + error[2]);

      fd[1] = fd[0];
      fd[0] = alpha_1 * (d[0] + d[1]) - alpha_2 * fd[1];

      output += fd[0];
      break;
    }
  }

  return output;
}
