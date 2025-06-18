#include <PIDControllerESP32.h>

// PID IIR: usa ecuación recursiva como filtro digital (más eficiente)
PIDControllerESP32 pid(1.0, 0.5, 0.1, 0.01, PID_IIR);

void setup() {
  Serial.begin(115200);
  pid.setSetpoint(1.65); // objetivo
}

void loop() {
  float sensor = analogRead(34) * 3.3 / 4095.0;
  pid.updateMeasurement(sensor);
  float output = pid.compute();
  Serial.println(output);
  delay(10);
}
