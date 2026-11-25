# Memoria Técnica: Proyecto Seguidor de Línea Pro v1 (ESP32-C3 + CD74HC4067 + 2x QTR-8A Chinas + TB6612)

## 1. Mapeo de Pines (KiCad Schematic `v1.kicad_sch`)
- **Microcontrolador:** ESP32-C3 SuperMini (`esp32:esp32:nologo_esp32c3_super_mini`, puerto `COM5`, USB CDC On Boot = Enabled).
- **Driver de Motores (U1 - TB6612):**
  - `AI1` (Motor Izq / A) -> **GPIO 1**
  - `AI2` (Motor Izq / A) -> **GPIO 2**
  - `BI1` (Motor Der / B) -> **GPIO 3**
  - `BI2` (Motor Der / B) -> **GPIO 8** (comparte con el LED azul de la placa)
  - `PWMA`, `PWMB`, `STBY` -> **3.3V / VCC** (Siempre habilitados por hardware en esquemático)
  - `VM` -> Batería 12V (`BT1`)
  - **Método de Control:** Como `PWMA` y `PWMB` están fijas a VCC, el control de velocidad y dirección se hace por modulación PWM directa sobre los pares `(AI1, AI2)` y `(BI1, BI2)` usando hardware LEDC a **20 kHz** (ultrasónico/silencioso, resolución 8 bits: 0 a 255):
    - *Adelante:* Motor B (`BI1 = PWM, BI2 = 0`); Motor A tiene polaridad física invertida (`AI1 = 0, AI2 = PWM`)
    - *Reversa:* Motor B (`BI1 = 0, BI2 = PWM`); Motor A (`AI1 = PWM, AI2 = 0`)
    - *Freno:* `IN1 = 255`, `IN2 = 255`
    - *Paro (libre):* `IN1 = 0`, `IN2 = 0`
- **Multiplexor Analógico (U5 - CD74HC4067):**
  - `SIG` / `COM` (Pin 1) -> **GPIO 0** (Entrada ADC)
  - `S0` (Pin 10) -> **GPIO 20** (conexión física real en placa)
  - `S1` (Pin 11) -> **GPIO 10**
  - `S2` (Pin 14) -> **GPIO 7**
  - `S3` (Pin 13) -> **GPIO 6**
  - `EN` / `~E` (Pin 15) -> **GND** (Siempre habilitado)
- **Barras de Sensores (U3 y U4 - 2x QTR-8A Chinas, 16 canales en total):**
  - `LED_ON` (Emisores IR) -> **GPIO 4** (`HIGH` para encender)
  - Canales `CH00..CH07` -> Barra 1 (`U3_LED8` a `U3_LED1` en orden descendente)
  - Canales `CH08..CH15` -> Barra 2 (`U4_LED8` a `U4_LED1` en orden descendente)

## 2. Regla Crítica de Hardware: Anti-Saturación ADC en ESP32-C3 (`4095` Fijo)
- **Causa:** El ADC del ESP32-C3 con `ADC_11db` satura (`4095`) a los **`2922 mV` (`2.92 V`)**. Las barras de sensores chinas tienen resistencias pull-up internas (~15 kΩ a 3.3V) que mantienen el voltaje en reposo entre `3.0 V` y `3.3 V`. Sin carga, el ADC lee `4095` fijo todo el tiempo y no detecta variaciones.
- **Detalle del Core Arduino ESP32 v3.x:** Llamar a `pinMode(0, INPUT_PULLDOWN)` antes de `analogRead(0)` NO funciona porque `analogRead()` reconfigura el pin a modo `ANALOG` y desactiva el pull-down antes de tomar la muestra.
- **Solución Obligatoria en Código:**
  Incluir `#include "driver/gpio.h"`, inicializar el ADC primero y luego forzar el pull-down interno (~45 kΩ a GND) directamente en el registro de hardware (`gpio_pulldown_en`):
  ```cpp
  #include "driver/gpio.h"

  // En setup():
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogReadMilliVolts(PIN_SIG);          // 1. Registra el pin en el bus ADC (periman)
  gpio_pullup_dis((gpio_num_t)PIN_SIG);   // 2. Desactiva pull-up
  gpio_pulldown_en((gpio_num_t)PIN_SIG);  // 3. Activa pull-down de 45k por hardware sin romper el ADC
  ```
  Esto forma un divisor de voltaje con la resistencia de las barras, bajando el reposo a **`~2483 mV`** (dentro de la región lineal del ADC) y permitiendo medir variaciones finas con `analogReadMilliVolts(PIN_SIG)`.
