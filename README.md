# Seguidor de Línea Pro v1 (ESP32-C3 + 16 Sensores + EDF 2S)

Proyecto de robot seguidor de línea velocista de 16 sensores basado en **ESP32-C3 SuperMini**, multiplexor analógico **CD74HC4067**, dos barras de sensores **QTR-8A** (versión china) dispuestas en **V (22°)**, driver de motores **TB6612FNG**, motores **JSumo 12V 3600 RPM**, llantas de silicón **Ø20 mm** y turbina **EDF 30 mm 14000KV** con alimentación independiente **2S (7.4V)**.

> **100% Libre de Wi-Fi / Bluetooth / RF:** Todo el proyecto opera de forma autónoma para cumplir el reglamento **RoboTiger Velocista Pro**. La calibración en banco y el ajuste fino de parámetros se realizan por **USB Serial (115200 baud)** con guardado automático en **Flash NVS (`Preferences`)**.

---

## Estructura del Repositorio

| Carpeta / Archivo | Descripción |
| :--- | :--- |
| `seguidor_pro/seguidor_pro.ino` | **Firmware único** de competencia (500 Hz exactos, `dt = 2 ms`), consola USB Serial de calibración/diagnóstico y persistencia NVS. |
| `test_turbina_nano/` | Sketch auxiliar de prueba PWM para ESC de turbina en Arduino Nano. |
| `v1/` | Esquemático y PCB base en KiCad (`v1.kicad_sch`, `v1.kicad_pcb`). |
| `v1_enrutado/` | PCB enrutado y vistas 3D de referencia. |

---

## Especificaciones de Hardware y Mapeo de Pines

- **Microcontrolador:** ESP32-C3 SuperMini (`esp32:esp32:nologo_esp32c3_super_mini`, `USB CDC On Boot = Enabled`).
- **Tracción:** 2× motores **JSumo 12V 3600 RPM**, rines de aluminio **Ø12 × 33 mm** (eje 3 mm) + gomas de silicón **Ø20 × 28 mm** (`Ø externa = 20 mm`).
- **Alimentación Principal (Tracción y Lógica):** LiPo **12V 750 mAh** directo a `VM` del TB6612 + regulador **step-down 12V→5V 3A** para alimentar el ESP32-C3.
- **Sistema Aerodinámico (Independiente):** Turbina **EDF 30 mm 14000KV** (~224 gf / 2.20 N de empuje máx.) + **ESC 35A** alimentados por batería aparte **7.4V (2S) 450 mAh** (cable rojo BEC del ESC desconectado; **GND común** con la placa).

| Subsistema | Señal / Función | Pin ESP32-C3 | Notas |
| :--- | :--- | :--- | :--- |
| **TB6612 (LEDC 20 kHz, 8 bits)** | `AI1` / `AI2` (Motor Izq A) | **GPIO 1**, **GPIO 2** | `PWMA`, `PWMB`, `STBY` fijos a `3.3V` |
| **TB6612 (LEDC 20 kHz, 8 bits)** | `BI1` / `BI2` (Motor Der B) | **GPIO 3**, **GPIO 8** | `GPIO 8` comparte LED azul integrado |
| **CD74HC4067 (Mux 16 ch)** | `SIG` / `COM` (Entrada ADC) | **GPIO 0** | Pull-down interno 45 kΩ forzado por registro |
| **CD74HC4067 (Mux 16 ch)** | `S0`, `S1`, `S2`, `S3` | **GPIO 6, 7, 10, 20** | `EN` (`~E`) fijo a `GND` |
| **Barras 2× QTR-8A (V 22°)** | `LED_ON` (Emisores IR) | **GPIO 4** | `HIGH` activo; `CH00..07` Barra 1, `CH08..15` Barra 2 |
| **Turbina EDF (LEDC 50 Hz)** | Señal PWM ESC (1000–2000 µs) | **GPIO 5** | Resolución 14 bits; armado seguro en `1000 µs` (0%) |
| **Arrancador Juez RRT** | Señal `GO` (`5V, GND, GO, RDY`) | **GPIO 21** | `INPUT_PULLDOWN`, antirrebote 3 lecturas a 500 Hz |
| **Calibración en Pista** | Botón físico `BOOT_CAL` | **GPIO 9** | `INPUT_PULLUP` (botón `BOOT` integrado) |

---

## Detalles Críticos de Ingeniería

1. **Anti-Saturación ADC en ESP32-C3 (`4095` Fijo):**
   El ADC con `ADC_11db` satura a **`2922 mV`**, mientras que las barras QTR-8A chinas tienen pull-ups (~15 kΩ a 3.3V) que las dejan entre `3.0 V` y `3.3 V`. En Arduino ESP32 v3.x, `analogReadMilliVolts()` reconfigura el pin y borra `pinMode(..., INPUT_PULLDOWN)`. Por ello el código ejecuta en `setup()`:
   - `analogReadMilliVolts(PIN_SIG);`
   - `gpio_pullup_dis((gpio_num_t)PIN_SIG);`
   - `gpio_pulldown_en((gpio_num_t)PIN_SIG);`
   Esto forma un divisor de voltaje (~`2483 mV` en reposo) dentro de la zona lineal del ADC.

2. **USB CDC No Bloqueante en Pista (`dt = 2 ms`):**
   Sin laptop conectada, escribir al puerto serie en ESP32-C3 puede bloquear el lazo de 500 Hz. El firmware usa `Serial.setTxTimeoutMs(0)`, verificación previa `Serial.availableForWrite()` y **pausa la telemetría periódica automáticamente durante la carrera**.

3. **Constantes Calibradas por Simulación Física (JSumo 3600 RPM + Ø20 mm + EDF 2S, 7/7 Escenarios):**
   Valores por defecto cargados cuando NVS está vacío (o al ejecutar `reset`):
   - `vr = 95` (velocidad recta), `vc = 58` (velocidad mínima en curva), `kf = 175` (freno en curva), `K_FRENO_DYN = 3.0`, `BRAKE_CAP = -195.0`, `MAX_PID = 175.0`.
   - `kpb = 0.34`, `kpc = 0.56` (ganancia cuadrática), `kd = 0.014` (`Kd_step = 7.00` a 2 ms, `ALPHA_D = 0.20`).
   - `tb = 50%` (recta), `tc = 80%` (curva cerrada), rampa de entrada 200 ms, decaimiento 160 ms (limitado a `[20%, 100%]` para no apagar la turbina en carrera).

---

## Compilación y Carga

- **Placa en Arduino IDE:** `NOLOGO ESP32C3 Super Mini` (`esp32:esp32:nologo_esp32c3_super_mini`)
- **USB CDC On Boot:** `Enabled` (`CDCOnBoot=cdc`)
- **Puerto:** `COM5` (o el COM asignado al conectar por USB-C)
- **Archivo único a abrir:** `seguidor_pro/seguidor_pro.ino`

---

## Calibración y Ajuste Fino por USB Serial (`115200 baud`)

Al conectar el robot a la PC (Monitor Serial a **115200**, funciona con `"Nueva línea"` o `"Sin ajuste de línea"`), dispones de consola interactiva sin recompilar:

| Comando | Acción |
| :--- | :--- |
| `c` | Autocalibra la base de los 16 sensores (percentil 75, ~1.2 s). Equivalente en pista al botón **`BOOT` (GPIO 9)**. |
| `v` | Activa / pausa la telemetría en vivo cada 250 ms (`S:[########|........] Err:+000 Turb: 0% GO:0`). |
| `t` (o `m`) | Test completo de motores (cuenta regresiva 3 s: A adelante, B adelante, ambos, reversa, giro en eje). |
| `1` / `2` | Prueba rápida 1 s solo Motor A (Izq) o Motor B (Der). |
| `u` | Prueba segura de turbina EDF al **25% durante 1.5 s** (sujetar robot; desconectar en `0%`). |
| `l` | Activa / desactiva el seguidor en banco (500 Hz + turbina). **Usar con el robot levantado o en pista controlada.** |
| `s` | **PARO DE EMERGENCIA inmediato** (detiene motores y apaga turbina al 0%, incluso en medio de una prueba o señal `GO`). |
| `p` o `?` | Muestra los parámetros actuales guardados en Flash NVS y el resumen de comandos. |
| `vr=95`, `vc=58`, `kf=175` | Ajusta velocidades / freno y guarda automáticamente en **Flash NVS**. |
| `kpb=0.34`, `kpc=0.56`, `kd=0.014` | Ajusta ganancias del PID dinámico y guarda automáticamente en **Flash NVS**. |
| `tb=50`, `tc=80` | Ajusta potencia base/curva de la turbina EDF (`20%..100%`) y guarda en **Flash NVS**. |
| `reset` | Restaura todas las constantes calibradas por defecto (`3600 RPM / Ø20 mm`) en Flash NVS. |

### Secuencia en Pista (Competencia)
1. Encender baterías (ESP32 arma el ESC de la turbina a `1000 µs` en `1 s` y realiza una autocalibración inicial).
2. Colocar el robot sobre el fondo de la pista y presionar **`BOOT` (GPIO 9)** si se requiere recalibrar la luz del coliseo (toma ~1.2 s, dentro de los 15 s previos).
3. Conectar o esperar señal positiva **`GO` (`GPIO 21`)** del módulo arrancador RRT del juez: el robot arranca en el lazo determinista de 500 Hz con turbina encendida y se detiene con freno activo cuando `GO` vuelve a `LOW`.
