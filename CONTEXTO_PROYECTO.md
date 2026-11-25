# Contexto Técnico y Estado Actual — Chavalin (Seguidor de Línea Pro v1)

**Última actualización:** 2026-11-25  
**Repositorio:** `https://github.com/ing-melcat/Chavalin.git` (rama `main`)

---

## 1. Hardware Real del Robot (Verificado Físicamente)

- **Chasis y dimensiones reales:**
  - **Peso total:** ~`300 g` (con ambas baterías LiPo, ESC y EDF montados).
  - **Tamaño:** `20 cm` de largo × `19 cm` de ancho (entre llantas).
  - **Geometría de sensores:** Dos barras chinas QTR-8A (16 canales `I0..I15`) montadas en **V invertida (`^`) cerrada (~110°–120°)** al frente del chasis.
- **Microcontrolador:** ESP32-C3 SuperMini (`esp32:esp32:nologo_esp32c3_super_mini`, `CDCOnBoot=cdc`, puerto `COM5`).
- **Tracción y driver:**
  - **Motores:** 2× JSumo 12V 3600 RPM con llantas de silicón Ø20 mm.
  - **Driver:** Módulo TB6612FNG (`U1`) con disipador + **capacitor electrolítico `470 µF / 16 V`** soldado entre `VM` (Pin 1) y `GND` (Pin 3) para absorber picos inductivos. *(Se recomienda cambiar a `470 µF / 25 V` en cuanto haya disponible).*
  - **Batería tracción:** LiPo 3S (`12.6 V` cargada, 750 mAh) + step-down 12V→5V para lógica.
  - **Polaridad física verificada (corregida por software en `setMotorA`):**
    - `AI1` (`GPIO 1`) / `AI2` (`GPIO 2`) → **Motor A (Izquierdo):** polaridad física invertida → **Adelante = `AI1=0, AI2=PWM`**, Reversa = `AI1=PWM, AI2=0`.
    - `BI1` (`GPIO 3`) / `BI2` (`GPIO 8`) → **Motor B (Derecho):** **Adelante = `BI1=PWM, BI2=0`**, Reversa = `BI1=0, BI2=PWM`. *(Nota: `GPIO 8` es pin de strapping del ESP32-C3; produce un brinco muy breve ~50–100 ms al encender porque `STBY`/`PWMA`/`PWMB` están fijos a `3.3 V` en PCB v1).*
    - PWM LEDC a **20 kHz, 8 bits (`0..255`)**.
- **Multiplexor y sensores (16 canales activos `I0..I15`):**
  - CD74HC4067 (`U5`): `SIG` = `GPIO 0`, `EN` = `GND`.
  - **Orden físico real de selectores en PCB:** `S0 = GPIO 20`, `S1 = GPIO 10`, `S2 = GPIO 7`, `S3 = GPIO 6`.
  - `LED_ON` = `GPIO 4` (`HIGH`).
  - **Regla anti-saturación ADC ESP32-C3 (`4095` fijo):** En `setup()` y antes de leer, registrar `analogReadMilliVolts(PIN_SIG)` y forzar `gpio_pullup_dis(GPIO_NUM_0)` + `gpio_pulldown_en(GPIO_NUM_0)` (~45 kΩ interno → reposo lineal `~2545 mV`).
  - **Estado actual de barras:** Los 16 canales (`I0..I15`) ya están **100% activos y estables** (`~2546..2557 mV` en reposo).
- **Turbina EDF (actualmente desactivada a `0%` para afinar PID base):**
  - QX-Motor 30 mm 14000KV + ESC 35A en **`GPIO 5`** (LEDC 50 Hz, 14 bits, `1000..2000 µs`).
  - Alimentada con LiPo 2S independiente (`7.4 V`, 450 mAh, cable rojo BEC desconectado, `GND` común).
  - Probada físicamente al `25%` (`1250 µs`) con éxito.

---

## 2. Arquitectura de Firmware y Flujo Pits → Carrera

### Sketches principales
1. **`seguidor_web/seguidor_web.ino` + `seguidor_web/web_ui.h`** *(cargado actualmente en el ESP32-C3)*:
   - **Wi-Fi AP:** Red **`Chavalin`**, contraseña **`Chavalin`**, panel en **`http://192.168.4.1`**.
   - **Lazo activo a `4 ms` (`250 Hz`)** con lectura rápida de sensores durante la corrida, telemetría web espaciada a `600 ms` en carrera (bajo *jitter* de Wi-Fi) y **`PARO (E-STOP)`** instantáneo.
   - **Calibración por barrido (`Calibrar Línea (3.5s)`):** Captura `cal_min_mv[16]` y `cal_max_mv[16]` pasando las barras sobre blanco y negro, normaliza individualmente cada canal `0..1000` y guarda en Flash NVS (`cal_ok`, `cmin`, `cmax`).
   - **Arranque desde web sin control remoto IR:** `Salida 3s` (cuenta regresiva de 3 s) y `Arrancar Ya`. `PIN_GO` (`GPIO 21`) está deshabilitado en modo Web para evitar falsos disparos o apagado de Wi-Fi.
   - **Sincronización automática a `seguidor_pro`:** Al tocar **`Aplicar PID`** en el perfil **`Ajuste Web + Sync Carrera`**, guarda los mismos parámetros (`vr, vc, kf, kpb, kpc, kd, tb, tc`) y calibración en Flash NVS (`namespace "chavalin"`).
2. **`seguidor_pro/seguidor_pro.ino`** *(firmware oficial de competencia, 100% Cero Wi-Fi)*:
   - No incluye librerías Wi-Fi ni radio RF (cumple reglamento de competencia).
   - Lazo determinista a **`2 ms` (`500 Hz`)** y hereda automáticamente de Flash NVS (`"chavalin"`) la configuración PID y la calibración por barrido guardadas desde `seguidor_web`.
   - **Arranque en pista oficial:** Botón `BOOT` (`GPIO 9`) toque corto = calibrar 3.5 s; mantener `BOOT` 1 s = cuenta regresiva 3 s + arranque 500 Hz (otro toque frena/para); también soporta módulo RRT del juez en `PIN_GO` (`GPIO 21`).
   - Límite de `kd_phys` ampliado hasta **`0.50`** para chasis pesados de 300 g.
3. **`test_motores/test_motores.ino`**:
   - Modo manual por Serial (`115200` baud: `1/2` Izq, `3/4` Der, `w/r` ambos, `t` auto, `+/-` PWM, `s` paro) con rampa suave `irSuaveMotores()` y polaridad de Motor A corregida.

### Protecciones activas para el TB6612FNG (en `seguidor_web` y `seguidor_pro`)
- **Paro inmediato en línea 1 de `setup()`:** `ledcAttach` + `stopMotors()` antes de `Serial.begin` / `delay(800)` para recortar el tirón de arranque.
- **Curva de aceleración inicial:** `MOT_RAMPA_S = 0.35 s`.
- **Limitador *slew-rate* ágil + paso obligatorio por `0` (`slewUnMotor`):** `MOT_SLEW_PWM_S = 2800 PWM/s` (~35 ms); impide cruzar directamente de `+PWM` a `-PWM` sin pasar por `0`.
- **Escalado proporcional con `VEL RECTA`:** `pid_cap`, `brake_floor`, `pwm_max_fwd` (tope `230`) y `pwm_min_rev` (tope `-85`) se escalan según `VEL RECTA`, evitando que a bajas velocidades mande `-255`.
- **Freno en 2 fases al detener:** `50 ms` coast `(0,0)` → `150 ms` freno activo → paro libre.
- **Corte automático por robot levantado / fuera de pista:** Si `>= 13` sensores leen `> 650` durante `250 ms` continuos, corta motores y turbina.
- **Turbina en `0%` por defecto** (`t_off1` en NVS) y subida limitada (`TURB_SLEW_PCT_S = 250 %/s`) cuando se vuelva a habilitar.

---

## 3. Resultados de Pruebas en Pista (Estado Actual)

- **Configuración actual guardada en NVS (Paso 1 — completó una vuelta completa limpiamente):**
  - `VEL RECTA = 70`, `VEL CURVA = 70`, `K FRENO = 0`
  - `KP BASE = 0.15`, `KP CURVA = 0.00`, `KD = 0.065`
  - `TURBINA = 0%`, `TURB CURVA = 0%`
- **Problema observado actualmente en curvas muy cerradas / circulares (confirmado con foto en pista):**
  - Debido a que las barras en V invertida forman un ángulo cerrado (~110°–120°), en curvas de radio pequeño ocurre que:
    1. **Ala paralela:** Toda una barra (`4–8` sensores seguidos en un lado) queda paralela sobre la línea negra; el centroide promedio reduce el error de `±750` a `~±400`, dando giro insuficiente.
    2. **Doble toque en horquilla:** Una punta de la V toca el tramo de salida y la otra ala toca el tramo de entrada simultáneamente; el centroide promedia ambos lados hacia `~0` y el carro continúa recto fuera de la curva.

---

## 4. Próximos Pasos Recomendados

1. **Implementar algoritmos profesionales de lectura para barra en V** en `calcularErrorLinea()` de `seguidor_web.ino` y `seguidor_pro.ino`:
   - **Segmentación por *clusters* contiguos:** Si hay 2 bloques negros separados (doble toque en horquilla/cruce), nunca promediarlos; seguir solo el bloque más cercano a `error_anterior`.
   - **Detector de ala paralela:** Si un bloque enciende `>= 4` sensores contiguos en una sola ala, usar el peso del **sensor más exterior activo** (en lugar del promedio) para forzar giro cerrado inmediato (`±750`).
   - ***Latch* de curva cerrada:** Mantener giro cerrado a `VEL CURVA` tras tocar punta extrema o ala paralela hasta recuperar los sensores centrales (`I5..I10`).
2. **Afinar curvas cerradas en Web:** Reactivar progresivamente `VEL CURVA` (`45`), `K FRENO` (`45`) y `KP CURVA` (`0.25–0.35`), luego escalar `VEL RECTA` (`85 → 100 → 120+`).
3. **Activar turbina EDF progresivamente** (`25% / 35%` → `35% / 55%`) una vez resueltas las curvas cerradas sin derrape.
4. **Cargar `seguidor_pro/seguidor_pro.ino` (500 Hz, Cero Wi-Fi)** para la competencia oficial (hereda automáticamente NVS).
