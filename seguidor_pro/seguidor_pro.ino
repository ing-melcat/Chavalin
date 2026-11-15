/*
  =============================================================================
  PROYECTO: SEGUIDOR DE LINEA PRO v1 - FIRMWARE DEFINITIVO DE COMPETENCIA
  =============================================================================
  Reglamento RoboTiger Velocista Pro - 100% Autónomo (CERO Wi-Fi / Radio / BT)
  Lazo de Control PID Dinámico: 500 Hz exactos (dt = 2 ms = 2000 us)

  Hardware (v1.kicad_sch):
    Microcontrolador: ESP32-C3 SuperMini (USB CDC On Boot = Enabled)
    Driver TB6612 (LEDC 20 kHz, 8 bits):
      - AI1 -> GPIO 1 (Motor Izq / A)
      - AI2 -> GPIO 2 (Motor Izq / A)
      - BI1 -> GPIO 3 (Motor Der / B)
      - BI2 -> GPIO 8 (Motor Der / B - comparte LED azul integrado)
      - PWMA, PWMB, STBY -> 3.3V / VCC (fijos por hardware)
    Multiplexor CD74HC4067 + 2x QTR-8A (16 canales, I0 Izq .. I15 Der):
      - SIG (COM) -> GPIO 0 (ADC con pull-down 45k por hardware anti-saturación)
      - S0..S3    -> GPIO 6, 7, 10, 20
      - LED_ON    -> GPIO 4 (HIGH)
    Turbina EDF Brushless ESC (LEDC 50 Hz, 14 bits, 1000..2000 us):
      - PWM_TURB  -> GPIO 5
    Módulo Arrancador del Juez (RRT: 5V, GND, GO, RDY):
      - GO        -> GPIO 21 (INPUT_PULLDOWN, antirrebote 3 lecturas HIGH)
    Botón Físico de Calibración en Pista (< 15 s):
      - BOOT_CAL  -> GPIO 9 (INPUT_PULLUP, botón BOOT integrado)
  =============================================================================
*/

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "driver/gpio.h"

// --- PINES DEL DRIVER TB6612 (LEDC 20 kHz, 8 bits) ---
#define PIN_MOT_A1    1   // AI1: Motor Izquierdo (A)
#define PIN_MOT_A2    2   // AI2: Motor Izquierdo (A)
#define PIN_MOT_B1    3   // BI1: Motor Derecho (B)
#define PIN_MOT_B2    8   // BI2: Motor Derecho (B) - Comparte LED azul de la placa

const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 8;

// --- PINES DEL MULTIPLEXOR CD74HC4067 Y SENSORES IR ---
#define PIN_SIG       0   // ADC (COM/SIG del CD74HC4067)
#define PIN_S0        6   // Selector S0
#define PIN_S1        7   // Selector S1
#define PIN_S2       10   // Selector S2
#define PIN_S3       20   // Selector S3
#define PIN_LED_ON    4   // Emisores IR

// --- PIN TURBINA EDF (ESC PWM 50 Hz, 14 bits) ---
#define PIN_TURBINA   5
const uint32_t ESC_PWM_FREQ = 50;
const uint8_t  ESC_PWM_RES  = 14; // 0..16383 (~1.22 us/paso)

// --- PINES DE COMPETENCIA (ARRANCADOR JUEZ RRT Y BOTÓN CALIBRACIÓN) ---
#define PIN_GO        21  // Señal GO del módulo RRT (INPUT_PULLDOWN)
#define PIN_BOOT_CAL   9  // Botón BOOT integrado ESP32-C3 (INPUT_PULLUP)

// --- GEOMETRÍA EN V (L0=150 mm, alpha=22 deg, I0=-750 Izq .. I15=+750 Der) ---
const int16_t PESOS_V[16] = {
  -750, -649, -548, -447, -347, -249, -154, -62,
    62,  154,  249,  347,  447,  548,  649,  750
};

// --- CONSTANTES CALIBRADAS POR SIMULACIÓN PARA 500 Hz (dt = 2 ms, Wi-Fi OFF) ---
const uint32_t LOOP_PERIOD_US    = 2000UL;
const float    DT_S              = 0.002f;

const int      VEL_RECTA         = 150;
const int      VEL_CURVA_MIN     = 68;
const int      K_FRENO           = 195;
const float    K_FRENO_DYN       = 3.0f;
const float    BRAKE_CAP         = -200.0f;
const float    TAU_SLOW          = 0.13f;
const float    ALPHA_SLOW        = DT_S / (TAU_SLOW + DT_S); // ~0.01515f @ 2 ms

const float    MAX_PID           = 160.0f;
const float    KP_BASE           = 0.34f;
const float    KP_CURVA          = 0.58f;
const float    KD_PHYS           = 0.015f;                   // Equiv. discreto Kd_step = 7.50f @ 2 ms
const float    TAU_D             = 0.008f;
const float    ALPHA_D           = DT_S / (TAU_D + DT_S);    // 0.20f @ 2 ms

const int      TURBINA_BASE_PCT  = 40;
const int      TURBINA_CURVA_PCT = 60;
const float    TAU_TURB_DECAY    = 0.16f;
const float    TURB_RAMPA_S      = 0.20f;                    // Rampa suave al activar (0% -> 40% en 200 ms)

// --- CALIBRACIÓN Y NORMALIZACIÓN DE SENSORES ---
const int RANGO_MAX_MV = 450;
const int UMBRAL_DELTA = 25;

uint16_t lecturas_mv[16];
uint16_t base_mv[16];
uint16_t norm_s[16];

// --- ESTADO DEL ROBOT Y LAZO DE CONTROL ---
enum EstadoRobot {
  DETENIDO = 0,
  COMPETENCIA_ACTIVA = 1,
  BANCO_SEGUIDOR = 2
};

EstadoRobot estado = DETENIDO;
uint8_t  go_high_count = 0;
bool     go_bloqueado_hasta_low = false;

bool     freno_activo = false;
uint32_t freno_fin_ms = 0;

int      motor_L = 0;
int      motor_R = 0;

float    error_actual = 0.0f;
float    error_anterior = 0.0f;
float    d_filt = 0.0f;
float    u_slow = 0.0f;

float    turb_cmd = 0.0f;
bool     turbina_rampa_done = false;
uint32_t ultimo_duty_turbina = 0xFFFFFFFFUL;

uint32_t last_loop_us = 0;
uint32_t ultimo_print_ms = 0;

// -----------------------------------------------------------------------------
// UTILIDADES AUXILIARES NO BLOQUEANTES
// -----------------------------------------------------------------------------
static inline float clampf(float val, float lo, float hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

void serialWriteNB(const char* text) {
  if (!text) return;
  size_t len = strlen(text);
  if (len == 0) return;
  if (Serial.availableForWrite() >= (int)len) {
    Serial.write((const uint8_t*)text, len);
  }
}

// -----------------------------------------------------------------------------
// CONTROL DE MOTORES TB6612 (PWM 20 kHz, 8 bits) Y TURBINA ESC (50 Hz, 14 bits)
// -----------------------------------------------------------------------------
void setMotorA(int speed) {
  speed = constrain(speed, -255, 255);
  motor_L = speed;
  if (speed > 0) {
    ledcWrite(PIN_MOT_A1, speed);
    ledcWrite(PIN_MOT_A2, 0);
  } else if (speed < 0) {
    ledcWrite(PIN_MOT_A1, 0);
    ledcWrite(PIN_MOT_A2, -speed);
  } else {
    ledcWrite(PIN_MOT_A1, 0);
    ledcWrite(PIN_MOT_A2, 0);
  }
}

void setMotorB(int speed) {
  speed = constrain(speed, -255, 255);
  motor_R = speed;
  if (speed > 0) {
    ledcWrite(PIN_MOT_B1, speed);
    ledcWrite(PIN_MOT_B2, 0);
  } else if (speed < 0) {
    ledcWrite(PIN_MOT_B1, 0);
    ledcWrite(PIN_MOT_B2, -speed);
  } else {
    ledcWrite(PIN_MOT_B1, 0);
    ledcWrite(PIN_MOT_B2, 0);
  }
}

void setMotors(int speedA, int speedB) {
  setMotorA(speedA);
  setMotorB(speedB);
}

void stopMotors() {
  setMotors(0, 0);
}

void brakeMotors() {
  motor_L = 0;
  motor_R = 0;
  ledcWrite(PIN_MOT_A1, 255);
  ledcWrite(PIN_MOT_A2, 255);
  ledcWrite(PIN_MOT_B1, 255);
  ledcWrite(PIN_MOT_B2, 255);
}

void setTurbina(float pct) {
  float pct_clamped = clampf(pct, 0.0f, 100.0f);
  uint32_t us = (uint32_t)lroundf(1000.0f + pct_clamped * 10.0f);
  uint32_t duty = (us * ((1UL << ESC_PWM_RES) - 1UL)) / 20000UL;
  if (duty != ultimo_duty_turbina || pct_clamped <= 0.0f) {
    ultimo_duty_turbina = duty;
    ledcWrite(PIN_TURBINA, duty);
  }
}

// -----------------------------------------------------------------------------
// LECTURA DE SENSORES, ANTI-SATURACIÓN ADC Y AUTOCALIBRACIÓN INTELIGENTE
// -----------------------------------------------------------------------------
void seleccionarCanalMux(uint8_t canal) {
  digitalWrite(PIN_S0, (canal & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_S1, (canal & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_S2, (canal & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_S3, (canal & 0x08) ? HIGH : LOW);
  delayMicroseconds(20);
}

void leerSensores() {
  // Mantener activo el pull-down de ~45k por hardware en PIN_SIG (anti-saturación 4095)
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_en((gpio_num_t)PIN_SIG);

  for (uint8_t pos = 0; pos < 16; pos++) {
    seleccionarCanalMux(pos);
    // 2 muestras rápidas para completar los 16 canales dentro del lazo de 2 ms (500 Hz)
    uint32_t suma = analogReadMilliVolts(PIN_SIG);
    suma += analogReadMilliVolts(PIN_SIG);
    lecturas_mv[pos] = (uint16_t)(suma / 2);

    // Auto-ajuste si el canal arrancó desconectado (<300 mV) y luego entra en reposo (>=2450 mV)
    if (base_mv[pos] < 300 && lecturas_mv[pos] >= 2450) {
      base_mv[pos] = lecturas_mv[pos];
    }

    if (base_mv[pos] < 300) {
      norm_s[pos] = 0;
    } else {
      int delta = abs((int)lecturas_mv[pos] - (int)base_mv[pos]);
      if (delta < UMBRAL_DELTA) {
        norm_s[pos] = 0;
      } else {
        long n = ((long)delta * 1000L) / RANGO_MAX_MV;
        norm_s[pos] = (uint16_t)constrain(n, 0, 1000);
      }
    }
  }
}

void calibrarBase() {
  stopMotors();
  setTurbina(0.0f);
  turb_cmd = 0.0f;
  turbina_rampa_done = false;
  serialWriteNB(">> Calibrando base de los 16 sensores...\n");

  uint32_t acum[16] = {0};
  for (int m = 0; m < 16; m++) {
    gpio_pullup_dis((gpio_num_t)PIN_SIG);
    gpio_pulldown_en((gpio_num_t)PIN_SIG);
    for (uint8_t pos = 0; pos < 16; pos++) {
      seleccionarCanalMux(pos);
      uint32_t suma = 0;
      for (int k = 0; k < 4; k++) {
        suma += analogReadMilliVolts(PIN_SIG);
      }
      acum[pos] += (suma / 4);
    }
    delay(4);
  }

  uint16_t ord[16];
  for (int i = 0; i < 16; i++) {
    base_mv[i] = (uint16_t)(acum[i] / 16);
    ord[i] = base_mv[i];
  }

  // Ordenar ascendente para obtener el percentil 75 (ord[12]) como referencia de blanco
  for (int i = 0; i < 15; i++) {
    for (int j = i + 1; j < 16; j++) {
      if (ord[j] < ord[i]) {
        uint16_t tmp = ord[i];
        ord[i] = ord[j];
        ord[j] = tmp;
      }
    }
  }
  uint16_t ref_blanco = ord[12];

  int canales_bajos = 0;
  for (int i = 0; i < 16; i++) {
    // Si la barra está alimentada y algún sensor pisaba la línea negra, corregir al blanco
    if (ref_blanco > 2200 && base_mv[i] > 300 && base_mv[i] < (ref_blanco - 60)) {
      base_mv[i] = ref_blanco;
    }
    if (base_mv[i] < 150) canales_bajos++;
  }

  // Reportar calibración por USB en líneas cortas no bloqueantes
  char buf[96];
  int n1 = snprintf(buf, sizeof(buf),
                    "Base I0..7 : %4u %4u %4u %4u %4u %4u %4u %4u\n",
                    base_mv[0], base_mv[1], base_mv[2], base_mv[3],
                    base_mv[4], base_mv[5], base_mv[6], base_mv[7]);
  if (n1 > 0 && Serial.availableForWrite() >= n1) {
    Serial.write((const uint8_t*)buf, n1);
  }
  int n2 = snprintf(buf, sizeof(buf),
                    "Base I8..15: %4u %4u %4u %4u %4u %4u %4u %4u (ref=%u)\n",
                    base_mv[8], base_mv[9], base_mv[10], base_mv[11],
                    base_mv[12], base_mv[13], base_mv[14], base_mv[15], ref_blanco);
  if (n2 > 0 && Serial.availableForWrite() >= n2) {
    Serial.write((const uint8_t*)buf, n2);
  }
  if (canales_bajos == 16) {
    serialWriteNB("[!] ALERTA HARDWARE: 16 canales ~0 mV -> Revisar SIG(0), EN o 3.3V.\n");
  }

  leerSensores();
  error_actual = 0.0f;
  error_anterior = 0.0f;
  d_filt = 0.0f;
  u_slow = 0.0f;
  last_loop_us = micros();
}

// Centroide con PESOS_V[16] y memoria de borde (±750 si los 16 sensores pierden la línea)
float calcularErrorLinea(bool en_carrera) {
  long suma_ponderada = 0;
  long suma_pesos = 0;

  for (int i = 0; i < 16; i++) {
    if (norm_s[i] > 0) {
      suma_ponderada += (long)PESOS_V[i] * norm_s[i];
      suma_pesos += norm_s[i];
    }
  }

  if (suma_pesos > 0) {
    return (float)suma_ponderada / (float)suma_pesos;
  }
  if (!en_carrera) {
    return 0.0f;
  }
  if (fabsf(error_anterior) > 10.0f) {
    return (error_anterior > 0.0f) ? 750.0f : -750.0f;
  }
  return error_anterior;
}

// -----------------------------------------------------------------------------
// TRANSICIONES DE CARRERA, FRENADO Y CONTROL PID DINÁMICO (500 Hz)
// -----------------------------------------------------------------------------
void iniciarCarrera(EstadoRobot nuevo_estado) {
  freno_activo = false;
  estado = nuevo_estado;
  d_filt = 0.0f;
  u_slow = 0.0f;
  turb_cmd = 0.0f;
  turbina_rampa_done = false;
  error_actual = calcularErrorLinea(true);
  error_anterior = error_actual; // Evita pico derivativo en el primer ciclo

  if (nuevo_estado == COMPETENCIA_ACTIVA) {
    serialWriteNB(">> [COMPETENCIA] GO detectado -> 500 Hz + Turbina ON.\n");
  } else {
    serialWriteNB(">> [BANCO] Seguidor ON (500 Hz + Turbina).\n");
  }
}

void detenerPorJuez() {
  brakeMotors();
  setTurbina(0.0f);
  estado = DETENIDO;
  freno_activo = true;
  freno_fin_ms = millis() + 300;
  d_filt = 0.0f;
  u_slow = 0.0f;
  turb_cmd = 0.0f;
  turbina_rampa_done = false;
  serialWriteNB(">> [COMPETENCIA] GO LOW -> Freno inmediato + Turbina 0%.\n");
}

void detenerBanco() {
  stopMotors();
  setTurbina(0.0f);
  estado = DETENIDO;
  freno_activo = false;
  d_filt = 0.0f;
  u_slow = 0.0f;
  turb_cmd = 0.0f;
  turbina_rampa_done = false;
}

void ejecutarControlPID500Hz() {
  // 1. Ganancia cuadrática y D filtrada invariante a dt_s (tau_d = 0.008 s)
  float u_err = fminf(1.0f, fabsf(error_actual) / 750.0f);
  float kp_din = KP_BASE + KP_CURVA * (u_err * u_err);
  float de_dt = (error_actual - error_anterior) / DT_S;
  d_filt = ALPHA_D * de_dt + (1.0f - ALPHA_D) * d_filt;

  float pid_raw = kp_din * error_actual + KD_PHYS * d_filt;
  float salida_pid = clampf(pid_raw, -MAX_PID, MAX_PID);

  // 2. Freno predictivo en entrada brusca a horquilla (tau_slow = 0.13 s)
  u_slow += ALPHA_SLOW * (u_err - u_slow);
  float u_jump = fmaxf(0.0f, u_err - u_slow);
  float vel_base = fmaxf((float)VEL_CURVA_MIN, (float)VEL_RECTA - (float)K_FRENO * u_err);
  float vel_din = fmaxf(BRAKE_CAP, vel_base - K_FRENO_DYN * (float)K_FRENO * u_jump);

  // 3. Mezcla diferencial TB6612 (Motor A Izq +pid, Motor B Der -pid)
  int pwm_A = constrain((int)lroundf(vel_din + salida_pid), -255, 255);
  int pwm_B = constrain((int)lroundf(vel_din - salida_pid), -255, 255);
  setMotors(pwm_A, pwm_B);

  // 4. Turbina EDF siempre activa durante la carrera:
  //    Rampa suave al activar (0 -> TURBINA_BASE_PCT en 200 ms),
  //    luego ataque inmediato al subir y decaimiento exponencial de 160 ms al salir de curva.
  float turb_tgt = (float)TURBINA_BASE_PCT + (float)(TURBINA_CURVA_PCT - TURBINA_BASE_PCT) * u_err;
  if (!turbina_rampa_done) {
    turb_cmd += (float)TURBINA_BASE_PCT * (DT_S / TURB_RAMPA_S);
    if (turb_cmd >= turb_tgt) {
      turb_cmd = turb_tgt;
      turbina_rampa_done = true;
    }
  } else {
    if (turb_tgt >= turb_cmd) {
      turb_cmd = turb_tgt;
    } else {
      static const float kTurbDecay = expf(-DT_S / TAU_TURB_DECAY);
      turb_cmd = turb_tgt + (turb_cmd - turb_tgt) * kTurbDecay;
    }
  }
  setTurbina(turb_cmd);
}

// -----------------------------------------------------------------------------
// RUTINAS DE BANCO POR USB SERIAL (NO BLOQUEANTE EN LOOP)
// -----------------------------------------------------------------------------
void ejecutarTestMotores() {
  detenerBanco();
  serialWriteNB(">> [TEST MOTORES] Iniciando secuencia en 3 s...\n");
  delay(1000);
  serialWriteNB(">> 2...\n");
  delay(1000);
  serialWriteNB(">> 1...\n");
  delay(1000);

  serialWriteNB(">> 1/5 Motor A (Izq) adelante...\n");
  setMotors(130, 0);
  delay(1000);
  stopMotors();
  delay(300);

  serialWriteNB(">> 2/5 Motor B (Der) adelante...\n");
  setMotors(0, 130);
  delay(1000);
  stopMotors();
  delay(300);

  serialWriteNB(">> 3/5 Ambos adelante...\n");
  setMotors(100, 100);
  delay(1000);
  stopMotors();
  delay(300);

  serialWriteNB(">> 4/5 Ambos reversa...\n");
  setMotors(-100, -100);
  delay(1000);
  stopMotors();
  delay(300);

  serialWriteNB(">> 5/5 Giro eje (Izq)...\n");
  setMotors(-100, 100);
  delay(700);
  stopMotors();
  serialWriteNB(">> [TEST MOTORES] Completado.\n");
  last_loop_us = micros();
}

void procesarSerial() {
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    if (cmd == '\r' || cmd == '\n' || cmd == ' ') continue;

    if (cmd == 's' || cmd == 'S') {
      if (digitalRead(PIN_GO) == HIGH) {
        go_bloqueado_hasta_low = true;
      }
      detenerBanco();
      serialWriteNB(">> [ALTO] Motores y turbina detenidos.\n");
    } else if (estado == COMPETENCIA_ACTIVA) {
      // En carrera por RRT no se aceptan comandos de banco salvo 's'
      continue;
    } else if (cmd == 't' || cmd == 'T' || cmd == 'm' || cmd == 'M') {
      ejecutarTestMotores();
    } else if (cmd == 'c' || cmd == 'C') {
      calibrarBase();
    } else if (cmd == 'l' || cmd == 'L') {
      if (estado == BANCO_SEGUIDOR) {
        detenerBanco();
        serialWriteNB(">> [BANCO] Seguidor DESACTIVADO.\n");
      } else {
        iniciarCarrera(BANCO_SEGUIDOR);
      }
    } else if (cmd == '1') {
      detenerBanco();
      serialWriteNB(">> Motor A adelante 1s...\n");
      setMotorA(120);
      delay(1000);
      stopMotors();
      last_loop_us = micros();
    } else if (cmd == '2') {
      detenerBanco();
      serialWriteNB(">> Motor B adelante 1s...\n");
      setMotorB(120);
      delay(1000);
      stopMotors();
      last_loop_us = micros();
    }
  }
}

void verificarBotonCalibracion(uint32_t now_ms) {
  static bool     boot_prev = false;
  static bool     boot_handled = false;
  static uint32_t boot_press_ms = 0;

  bool pressed = (digitalRead(PIN_BOOT_CAL) == LOW);
  if (pressed) {
    if (!boot_prev) {
      boot_prev = true;
      boot_handled = false;
      boot_press_ms = now_ms;
    } else if (!boot_handled && (now_ms - boot_press_ms >= 35UL)) {
      boot_handled = true;
      calibrarBase();
    }
  } else {
    boot_prev = false;
    boot_handled = false;
  }
}

void imprimirTelemetriaSerialNB() {
  char bar[20];
  int idx = 0;
  for (int i = 0; i < 16; i++) {
    bar[idx++] = (norm_s[i] > 0) ? '#' : '.';
    if (i == 7) bar[idx++] = '|';
  }
  bar[idx] = '\0';

  char buf[96];
  int n = snprintf(buf, sizeof(buf),
                   "S:[%s] Err:%+4.0f Turb:%2.0f%% GO:%d\n",
                   bar, error_actual, turb_cmd, (digitalRead(PIN_GO) == HIGH) ? 1 : 0);
  if (n > 0 && Serial.availableForWrite() >= n) {
    Serial.write((const uint8_t*)buf, n);
  }
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0); // USB CDC estrictamente no bloqueante sin monitor conectado

  // 1. Inicializar Driver TB6612 (LEDC 20 kHz, 8 bits) y detener motores de inmediato
  ledcAttach(PIN_MOT_A1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_A2, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_B1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_B2, PWM_FREQ, PWM_RES);
  stopMotors();

  // 2. Inicializar Turbina ESC (LEDC 50 Hz, 14 bits) con armado seguro a 1000 us
  ledcAttach(PIN_TURBINA, ESC_PWM_FREQ, ESC_PWM_RES);
  setTurbina(0.0f);

  // 3. Pines de competencia (Arrancador RRT GO y botón físico BOOT)
  pinMode(PIN_GO, INPUT_PULLDOWN);
  pinMode(PIN_BOOT_CAL, INPUT_PULLUP);

  // 4. Multiplexor CD74HC4067 y emisores IR
  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_LED_ON, OUTPUT);
  digitalWrite(PIN_LED_ON, HIGH);

  // Espera segura de armado ESC a 1000 us y estabilización de USB/sensores
  delay(1000);

  // 5. Inicialización ADC y regla obligatoria anti-saturación ESP32-C3
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogReadMilliVolts(PIN_SIG);
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_en((gpio_num_t)PIN_SIG);

  // 6. Autocalibración inteligente inicial (percentil 75 ord[12])
  calibrarBase();

  serialWriteNB("====================================================\n");
  serialWriteNB(" SEGUIDOR PRO v1 - COMPETENCIA 500 Hz (CERO WIFI)\n");
  serialWriteNB(" Comandos: 't' test | 's' alto | 'c' cal | 'l' run\n");
  serialWriteNB(" Pista: BOOT(9) recalibra | RRT GO(21) inicia/para\n");
  serialWriteNB("====================================================\n");

  last_loop_us = micros();
}

// -----------------------------------------------------------------------------
// LOOP PRINCIPAL (500 Hz = 2 ms)
// -----------------------------------------------------------------------------
void loop() {
  uint32_t now_us = micros();
  uint32_t now_ms = millis();

  // Paro inmediato fuera de ciclo si el juez baja PIN_GO tras arrancar
  if (estado == COMPETENCIA_ACTIVA && digitalRead(PIN_GO) == LOW) {
    detenerPorJuez();
  }

  // Liberar freno dinámico tras 300 ms en estado detenido
  if (freno_activo && (int32_t)(now_ms - freno_fin_ms) >= 0) {
    freno_activo = false;
    if (estado == DETENIDO) {
      stopMotors();
    }
  }

  // Procesar comandos USB Serial (no bloqueante)
  procesarSerial();

  // Botón físico BOOT (GPIO 9) para recalibrar en pista mientras está detenido
  if (estado == DETENIDO) {
    verificarBotonCalibracion(now_ms);
  }

  // Lazo determinista a 500 Hz (dt = 2000 us = 2 ms)
  if ((uint32_t)(now_us - last_loop_us) >= LOOP_PERIOD_US) {
    if ((uint32_t)(now_us - last_loop_us) >= (LOOP_PERIOD_US * 2UL)) {
      last_loop_us = now_us;
    } else {
      last_loop_us += LOOP_PERIOD_US;
    }

    // 1. Leer 16 sensores IR
    leerSensores();

    // 2. Antirrebote de PIN_GO (3 lecturas consecutivas HIGH a 500 Hz)
    bool go_high = (digitalRead(PIN_GO) == HIGH);
    if (go_high) {
      if (go_high_count < 3) go_high_count++;
      if (!go_bloqueado_hasta_low && go_high_count >= 3 && estado != COMPETENCIA_ACTIVA) {
        iniciarCarrera(COMPETENCIA_ACTIVA);
      }
    } else {
      go_high_count = 0;
      go_bloqueado_hasta_low = false;
      if (estado == COMPETENCIA_ACTIVA) {
        detenerPorJuez();
      }
    }

    // 3. Error con PESOS_V[16] y memoria de borde (±750)
    bool en_carrera = (estado == COMPETENCIA_ACTIVA || estado == BANCO_SEGUIDOR);
    error_actual = calcularErrorLinea(en_carrera);

    // 4. PID Dinámico + Freno Predictivo + Turbina EDF
    if (en_carrera) {
      ejecutarControlPID500Hz();
    }

    error_anterior = error_actual;
  }

  // Telemetría USB no bloqueante cada 250 ms únicamente cuando está detenido
  if (estado == DETENIDO && (now_ms - ultimo_print_ms >= 250UL)) {
    ultimo_print_ms = now_ms;
    imprimirTelemetriaSerialNB();
  }
}
