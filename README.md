# PIDControllerESP32

**Librería de control PID en tiempo discreto para ESP32**, diseñada para ser utilizada en proyectos como el control de un péndulo invertido u otros sistemas de control embebido.

Desarrollada por **Mateo_Javier** para el proyecto académico de la asignatura de Teoría de Control en la USC.

---

## Modos incluidos

Esta librería incluye tres implementaciones de PID discretos:

| Modo               | Descripción                                                                 |
|--------------------|------------------------------------------------------------------------------|
| `PID_BASIC`        | Implementación directa de P + I + D con diferencias finitas                 |
| `PID_IIR`          | Implementación tipo filtro IIR (más eficiente para sistemas embebidos)      |
| `PID_FILTERED_D`   | Igual que el básico, pero con filtro pasa-bajo en la derivada (menos ruido) |

---

## Instalación

1. Descarga o clona este repositorio.
2. Copia la carpeta `PIDControllerESP32/` en:
   ```
   Documentos/Arduino/libraries/
   ```
3. Reinicia el IDE de Arduino.

---

## Estructura de la librería

```
PIDControllerESP32/
├── PIDControllerESP32.h
├── PIDControllerESP32.cpp
├── library.properties
└── examples/
    ├── ExampleBasicPID/
    ├── ExampleIIRPID/
    └── ExampleFilteredDPID/
```

---

## Uso básico

### Incluir la librería y crear un objeto

```cpp
#include <PIDControllerESP32.h>

// Modo PID básico
PIDControllerESP32 pid(1.0, 0.5, 0.1, 0.01, PID_BASIC);
```

### Configurar y usar

```cpp
pid.setSetpoint(0.0);                 // Ángulo deseado
pid.updateMeasurement(medicion);     // Valor actual (ej. del sensor)
float salida = pid.compute();        // Calcula señal de control
```

---

## Pruebas con el péndulo invertido

Para conectar al sistema real:

- Lee el ángulo del péndulo desde un sensor (ej. MPU6050)
- Aplica la salida del PID al motor (ej. con `analogWrite()` o `ledcWrite()` en ESP32)
- Ajusta los parámetros `Kp`, `Ki`, `Kd` y `dt` hasta obtener estabilidad

---

## Ejemplos incluidos

Puedes encontrar ejemplos listos para ejecutar en:

```
Archivo > Ejemplos > PIDControllerESP32
```

- `ExampleBasicPID`: PID directo
- `ExampleIIRPID`: PID con filtro IIR
- `ExampleFilteredDPID`: PID con derivada filtrada

---
