# Seguidor de Línea Pro v1 (ESP32-C3 + 16 Sensores + Telemetría Wi-Fi)

Proyecto de robot seguidor de línea de 16 sensores basado en **ESP32-C3 SuperMini**, multiplexor analógico **CD74HC4067**, dos barras de sensores **QTR-8A** (versión china) y driver de motores **TB6612FNG**, con panel de control y telemetría en tiempo real por Wi-Fi.

---

## 1. Estructura del Proyecto

| Carpeta / Archivo | Descripción |
| :--- | :--- |
| `seguidor_web/seguidor_web.ino` | **Firmware principal activo**: lectura multiplexada de 16 sensores, control PID, manejo de motores TB6612 por LEDC (20 kHz), Access Point Wi-Fi y servidor HTTP nativo (`esp_http_server`). |
| `seguidor_web/web_ui.h` | Interfaz web embebida (`HTML/CSS/JS`) servida en `http://192.168.4.1`: barras normalizadas (`I0` a `I15`), voltaje en vivo (`mV`), tacómetros de motores, osciloscopio de comandos/error y ajuste PID en caliente. |
| `seguidor_pro/seguidor_pro.ino` | Versión anterior del seguidor sin servidor web. |
| `test_diag/test_diag.ino` | Sketch de diagnóstico eléctrico: verifica cortos/atascos en `S0..S3` y `LED_ON`, saturación del ADC y estabilidad del pull-down. |
| `test_sensores/`, `test_mux/`, `test_interactivo/` | Sketches auxiliares para pruebas puntuales de canales del multiplexor y lectura individual de sensores. |
| `v1/` | Proyecto KiCad base (`v1.kicad_sch`, `v1.kicad_pcb`, `v1.kicad_pro`), respaldo del PCB y `freerouting-2.0.1.jar`. |
| `v1_enrutado/` | PCB enrutado en KiCad (`v1_enrutado.kicad_pcb`), reporte DRC (`drc_report.json`) y vistas 3D (`vista_3d_top.png`, `vista_3d_bottom.png`). |
| `GEMINI.md` | Memoria técnica con el mapeo de pines del esquemático y la solución de hardware para el ADC. |

---

## 2. Hardware y Mapeo de Pines

- **Microcontrolador:** ESP32-C3 SuperMini (`esp32:esp32:nologo_esp32c3_super_mini`, `CDCOnBoot=cdc`, puerto `COM5`)
- **Driver de Motores (TB6612FNG):**
  - `AI1` (Motor Izquierdo) → **GPIO 1**
  - `AI2` (Motor Izquierdo) → **GPIO 2**
  - `BI1` (Motor Derecho) → **GPIO 3**
  - `BI2` (Motor Derecho) → **GPIO 8** *(comparte con LED azul de la placa)*
  - `PWMA`, `PWMB`, `STBY` → **3.3V / VCC** *(habilitados por hardware; control PWM directo a 20 kHz, 8 bits sobre `AI1/AI2` y `BI1/BI2`)*
  - `VM` → Batería 12V
- **Multiplexor Analógico (CD74HC4067) y 16 Sensores (2× QTR-8A):**
  - `SIG` / `COM` → **GPIO 0** (ADC)
  - `S0` → **GPIO 6**
  - `S1` → **GPIO 7**
  - `S2` → **GPIO 10**
  - `S3` → **GPIO 20**
  - `EN` (`~E`) → **GND** *(siempre habilitado)*
  - `LED_ON` (Emisores IR) → **GPIO 4** (`HIGH`)
- **Orden Lógico y Visual de Sensores:**
  - Izquierda → Derecha: **`I0`** (`CH00`, peso `-750`) hasta **`I15`** (`CH15`, peso `+750`).
- **Pines libres recomendados para expansión (ej. ESC de turbina):**
  - **`GPIO 5`** *(recomendado para señal PWM de 50 Hz; conectar también `GND` común y dejar el cable rojo `5V` del BEC desconectado/aislado si se alimenta por USB o regulador propio)*.
  - `GPIO 21` (`TX`) disponible; `GPIO 9` disponible pero compartido con botón `BOOT`.

---

## 3. Soluciones Técnicas Implementadas

1. **Anti-saturación del ADC del ESP32-C3 (`4095` fijo):**
   - Con `ADC_11db`, el ADC del ESP32-C3 satura a `2922 mV`, mientras las barras QTR-8A chinas mantienen ~`3.0–3.3 V` en reposo.
   - En `seguidor_web.ino` se registra primero el pin con `analogReadMilliVolts(PIN_SIG)` y después se fuerza el pull-down interno (~45 kΩ a GND) mediante `gpio_pullup_dis` + `gpio_pulldown_en`, bajando el voltaje en reposo a `~2480–2560 mV` dentro de la zona lineal.
2. **Estabilidad Wi-Fi en ESP32-C3 SuperMini:**
   - Access Point configurado en **Canal 8**, protocolo **802.11b/g** (`WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G`), potencia **`11 dBm`** y `WiFi.setSleep(false)`.
   - Servidor HTTP implementado con **`esp_http_server.h`** (HTTP/1.1 Keep-Alive + `lru_purge_enable = true`), evitando el agotamiento de sockets `TIME_WAIT`/`FIN_WAIT_2`.
3. **USB Serial no bloqueante:**
   - `Serial.setTxTimeoutMs(0)` y verificación `Serial.availableForWrite() >= 100` para evitar pausas en `loop()` cuando no hay monitor serial conectado en `COM5`.
4. **Calibración robusta:**
   - `calibrarBase()` calcula el percentil 75 de la barra como referencia de blanco por si algún sensor está sobre la línea negra al arrancar, y `leerSensores()` auto-ajusta canales que inicien desconectados al recuperar voltaje normal (`>= 2450 mV`).

---

## 4. Compilación y Uso

### Subir firmware (`arduino-cli` o Arduino IDE)
- **Placa:** `nologo_esp32c3_super_mini`
- **FQBN:** `esp32:esp32:nologo_esp32c3_super_mini:CDCOnBoot=cdc`
- **Archivo a abrir:** `seguidor_web/seguidor_web.ino` (carga automáticamente `web_ui.h`)

### Panel de Telemetría Web
1. Conéctate a la red Wi-Fi **`SeguidorPro-WiFi`** (contraseña: **`12345678`**).
2. Abre **`http://192.168.4.1`** en el navegador.
3. Coloca los sensores sobre la pista (blanco/fondo uniforme) y presiona **`Calibrar Sensores`**.
4. Ajusta `Base Speed`, `Kp`, `Ki`, `Kd` con **`Aplicar PID`** y usa **`Arrancar Seguidor`**, **`Test Motores`** o **`PARO (E-STOP)`**.
