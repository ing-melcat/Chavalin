/*
  =============================================================================
  PROYECTO: SEGUIDOR DE LINEA PRO v1 - TELEMETRÍA WEB + NVS + MODO COMPETENCIA
  =============================================================================
  - En Pits (Wi-Fi AP "SeguidorPro-WiFi" / http://192.168.4.1):
      * Lazo PID Dinámico a 12 ms (~83 Hz) con ganancias calibradas para 12 ms
      * Telemetría Web en vivo (16 sensores QTR-8A en V, tacómetros, osciloscopio)
      * Edición y persistencia en Flash NVS (Preferences namespace "chavalin")
  - En Competencia (RRT Receptor del Juez PIN_GO = GPIO 21 -> HIGH):
      * Apaga inmediatamente el servidor HTTP y el radio Wi-Fi (reglamento pág. 2)
      * Cambia el lazo a 2 ms (500 Hz) con ganancias calibradas para 2 ms
      * Turbina EDF activa por defecto (reglamento pág. 7, nunca 0% en arranque)
      * Al regresar PIN_GO a LOW: frena motores y apaga turbina a 0%
  - Calibración física en reposo: Botón BOOT (PIN_BOOT_CAL = GPIO 9)
  =============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <Preferences.h>
#include <math.h>
#include "driver/gpio.h"
#include "web_ui.h"

// --- CONFIGURACIÓN WIFI ACCESS POINT ---
const char* AP_SSID = "Chavalin";
const char* AP_PASS = "Chavalin";
const char* NVS_NAMESPACE = "chavalin";

httpd_handle_t server = NULL;

// --- PINES DRIVER TB6612 (PWM 20 kHz, 8 bits) ---
#define PIN_MOT_A1    1   // GPIO 1: AI1 (Motor Izq / A)
#define PIN_MOT_A2    2   // GPIO 2: AI2 (Motor Izq / A)
#define PIN_MOT_B1    3   // GPIO 3: BI1 (Motor Der / B)
#define PIN_MOT_B2    8   // GPIO 8: BI2 (Motor Der / B - comparte LED azul)

const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 8;

// --- PINES MULTIPLEXOR CD74HC4067 Y SENSORES ---
#define PIN_SIG       0   // GPIO 0: ADC COM/SIG (con pull-down hardware anti-saturación)
#define PIN_S0       20   // GPIO 20: S0 (orden físico invertido en placa/cableado)
#define PIN_S1       10   // GPIO 10: S1
#define PIN_S2        7   // GPIO 7:  S2
#define PIN_S3        6   // GPIO 6:  S3
#define PIN_LED_ON    4   // GPIO 4: Emisores IR

// --- PIN TURBINA (ESC BRUSHLESS PWM 50 Hz, 14 bits) ---
#define PIN_TURBINA   5   // GPIO 5: Señal PWM 50 Hz (1000-2000 us) para ESC
const uint32_t ESC_PWM_FREQ = 50;
const uint8_t  ESC_PWM_RES  = 14; // 0..16383 (~1.22 us por paso)

// --- PINES DE COMPETENCIA (RRT DEL JUEZ Y BOTÓN BOOT) ---
#define PIN_GO       21   // GPIO 21: RRT GO (INPUT_PULLDOWN, HIGH = Competencia)
#define PIN_BOOT_CAL  9   // GPIO 9:  Botón BOOT físico para calibrar en reposo

// --- GEOMETRÍA EN V (L0=150 mm, alpha=22 deg, I0=-750 Izq .. I15=+750 Der) ---
const int16_t PESOS_V[16] = {
  -750, -649, -548, -447, -347, -249, -154, -62,
    62,  154,  249,  347,  447,  548,  649,  750
};

// --- CONSTANTES FÍSICAS DEL PID DINÁMICO (@sim-math) ---
const float TAU_D          = 0.008f;  // Filtro derivativo (alpha_d = dt / (TAU_D + dt))
const float TAU_SLOW       = 0.13f;   // Envolvente predictiva de entrada a horquilla
const float TAU_TURB_DECAY = 0.16f;   // Decaimiento 160 ms de turbina al salir de curva
const float TURB_RAMPA_S   = 0.20f;   // Rampa suave al activar turbina (200 ms)
const float MOT_RAMPA_S    = 0.35f;   // Curva suave de aceleración inicial (350 ms)
const float MOT_SLEW_PWM_S = 2800.0f; // Límite slew-rate ágil (~35 ms) + paso obligatorio por 0
const float TURB_SLEW_PCT_S = 250.0f; // Límite de subida de turbina (%/s) en curva
const float MAX_PID        = 160.0f;  // Saturación de salida diferencial PID

const uint32_t LOOP_US_PITS = 4000UL;  // 250 Hz (Wi-Fi ON, bajo jitter)
const uint32_t LOOP_US_COMP = 2000UL;  // 500 Hz (Wi-Fi OFF)
const float    DT_PITS_S    = 0.004f;
const float    DT_COMP_S    = 0.002f;

struct PidConfig {
  int   vel_recta;
  int   vel_curva;
  int   k_freno;
  float k_freno_dyn;
  float brake_cap;
  float kp_base;
  float kp_curva;
  float kd_phys;
  int   turbina_base_pct;
  int   turbina_curva_pct;
};

// Constantes calibradas por @sim-math para 12 ms (Pits) y 2 ms (Competencia)
const PidConfig PITS_DEF = {
  95, 58, 160, 3.1f, -195.0f, 0.29f, 0.50f, 0.012f, 0, 0
};
const PidConfig COMP_DEF = {
  150, 68, 195, 3.0f, -200.0f, 0.34f, 0.58f, 0.015f, 40, 60
};

PidConfig cfg_pits = PITS_DEF;
PidConfig cfg_comp = COMP_DEF;

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// --- VARIABLES DE SENSORES Y CALIBRACIÓN ---
uint16_t lecturas_mv[16];     // Orden lógico Left->Right: índice 0 = I0 (Izq) .. 15 = I15 (Der)
uint16_t base_mv[16];
uint16_t norm_s[16];          // Normalizado 0 a 1000 para centroide y UI web
uint16_t cal_min_mv[16];
uint16_t cal_max_mv[16];
bool     calibrado_en_blanco = false;
bool     usa_barrido         = false;
volatile bool calibracion_barrido_activa = false;
uint32_t cal_fin_ms          = 0;
const int RANGO_MAX_MV = 600; // Rango de variación (mV) para escalar 0-1000
const int UMBRAL_DELTA = 35;  // Umbral de detección limpio sobre base individual

inline uint8_t canalFisico(uint8_t pos_izq_der) {
  return pos_izq_der;
}

// --- ESTADO DE CONTROL, MOTORES Y TURBINA ---
volatile bool seguidor_activo        = false;
volatile bool modo_competencia       = false;
volatile bool wifi_apagado           = false;
volatile bool turbina_manual         = false;
volatile bool cmd_start_pendiente    = false;
volatile bool cmd_start3_pendiente   = false;
volatile bool cuenta_regresiva_activa = false;
volatile bool cmd_stop_pendiente     = false;
volatile bool cmd_turb_on_pendiente  = false;
volatile bool cmd_turb_off_pendiente = false;
volatile bool test_motores_pendiente = false;
volatile bool calibrar_pendiente     = false;

uint32_t cuenta_fin_ms   = 0;
bool     turbina_rampa_done = false;
uint8_t  freno_fase      = 0;       // 0=inactivo, 1=coast 50ms anti-pico, 2=freno 150ms
uint32_t freno_fin_ms    = 0;
uint32_t todo_oscuro_ms  = 0;

int   turbina_pct    = 0;
float turb_cmd       = 0.0f;
float error_actual   = 0.0f;
float error_anterior = 0.0f;
float d_filt         = 0.0f;
float u_slow         = 0.0f;
float mot_rampa      = 0.0f;
float mot_cmd_L      = 0.0f;
float mot_cmd_R      = 0.0f;
int   motor_L        = 0;
int   motor_R        = 0;

uint32_t lastLoopUs  = 0;

// -----------------------------------------------------------------------------
// CONTROL DE MOTORES TB6612 Y TURBINA ESC
// -----------------------------------------------------------------------------
void setMotorA(int speed) {
  speed = constrain(speed, -255, 255);
  motor_L = speed;
  // Motor A (Izq) tiene polaridad física invertida en cableado: adelante = AI2 PWM
  if (speed > 0) {
    ledcWrite(PIN_MOT_A1, 0);
    ledcWrite(PIN_MOT_A2, speed);
  } else if (speed < 0) {
    ledcWrite(PIN_MOT_A1, -speed);
    ledcWrite(PIN_MOT_A2, 0);
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

void setMotors(int sL, int sR) {
  setMotorA(sL);
  setMotorB(sR);
}

static inline float slewUnMotor(float actual, float meta, float dt_s) {
  if ((actual > 1.0f && meta < 0.0f) || (actual < -1.0f && meta > 0.0f)) {
    meta = 0.0f; // Paso obligatorio por 0 antes de invertir polaridad
  }
  float max_step = MOT_SLEW_PWM_S * dt_s;
  float diff = meta - actual;
  if (diff > max_step)  return actual + max_step;
  if (diff < -max_step) return actual - max_step;
  return meta;
}

void stopMotors() {
  setMotors(0, 0);
  mot_cmd_L = 0.0f;
  mot_cmd_R = 0.0f;
  mot_rampa = 0.0f;
  d_filt = 0.0f;
  u_slow = 0.0f;
  error_anterior = 0.0f;
}

void brakeMotors() {
  motor_L = 0;
  motor_R = 0;
  ledcWrite(PIN_MOT_A1, 255);
  ledcWrite(PIN_MOT_A2, 255);
  ledcWrite(PIN_MOT_B1, 255);
  ledcWrite(PIN_MOT_B2, 255);
}

void setTurbinaFloat(float pct) {
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  turbina_pct = (int)lroundf(pct);
  float us = 1000.0f + pct * 10.0f;
  uint32_t max_duty = (1UL << ESC_PWM_RES) - 1UL;
  uint32_t duty = (uint32_t)lroundf((us * (float)max_duty) / 20000.0f);
  ledcWrite(PIN_TURBINA, duty);
}

void setTurbina(int pct) {
  setTurbinaFloat((float)constrain(pct, 0, 100));
}

void detenerTodo() {
  seguidor_activo = false;
  cuenta_regresiva_activa = false;
  calibracion_barrido_activa = false;
  turbina_manual = false;
  turbina_rampa_done = false;
  freno_fase = 0;
  todo_oscuro_ms = 0;
  stopMotors();
  setTurbina(0);
  turb_cmd = 0.0f;
}

void detenerConFreno() {
  bool estaba_corriendo = seguidor_activo;
  seguidor_activo = false;
  cuenta_regresiva_activa = false;
  calibracion_barrido_activa = false;
  turbina_manual = false;
  turbina_rampa_done = false;
  todo_oscuro_ms = 0;
  setTurbina(0);
  turb_cmd = 0.0f;
  stopMotors(); // Coast (0,0) primero para disipar corriente inductiva sin picos en VM
  if (estaba_corriendo) {
    freno_fase = 1;
    freno_fin_ms = millis() + 50UL;
  } else {
    freno_fase = 0;
  }
}

void iniciarCarrera(bool competencia) {
  freno_fase = 0;
  todo_oscuro_ms = 0;
  mot_rampa = 0.0f;
  mot_cmd_L = 0.0f;
  mot_cmd_R = 0.0f;
  cuenta_regresiva_activa = false;
  PidConfig cfg;
  portENTER_CRITICAL(&stateMux);
  cfg = competencia ? cfg_comp : cfg_pits;
  portEXIT_CRITICAL(&stateMux);

  modo_competencia = competencia;
  turbina_manual = false;
  error_anterior = error_actual;
  d_filt = 0.0f;
  u_slow = fminf(1.0f, fabsf(error_actual) / 750.0f);

  int t_base = cfg.turbina_base_pct;
  // Reglamento pág. 7: la turbina debe estar activa durante toda la carrera (nunca 0%)
  if (competencia && t_base < 20) t_base = COMP_DEF.turbina_base_pct;

  if (t_base <= 0) {
    turb_cmd = 0.0f;
    turbina_rampa_done = true;
    setTurbina(0);
  } else if (turb_cmd >= (float)t_base - 1.0f) {
    turb_cmd = (float)t_base;
    turbina_rampa_done = true;
    setTurbinaFloat(turb_cmd);
  } else {
    turbina_rampa_done = false;
  }
  seguidor_activo = true;
  lastLoopUs = micros();
}

void apagarRadioWiFi() {
  if (wifi_apagado) return;
  if (server) {
    httpd_stop(server);
    server = NULL;
  }
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifi_apagado = true;
}

void rampaBancoMotores(int tgtA, int tgtB, uint32_t hold_ms) {
  for (int i = 1; i <= 6; i++) {
    setMotors((tgtA * i) / 6, (tgtB * i) / 6);
    delay(25);
  }
  delay(hold_ms);
  for (int i = 4; i >= 0; i--) {
    setMotors((tgtA * i) / 5, (tgtB * i) / 5);
    delay(25);
  }
  stopMotors();
  delay(300);
}

void ejecutarTestMotores() {
  detenerTodo();
  Serial.println(">> [TEST MOTORES] Prueba suave anti-picos (Izq, Der, Adelante, Reversa)...");
  Serial.println(">> [TEST MOTORES] Motor IZQ adelante (+110)");
  rampaBancoMotores(110, 0, 450);
  Serial.println(">> [TEST MOTORES] Motor DER adelante (+110)");
  rampaBancoMotores(0, 110, 450);
  Serial.println(">> [TEST MOTORES] Ambos adelante (+100)");
  rampaBancoMotores(100, 100, 450);
  Serial.println(">> [TEST MOTORES] Ambos reversa (-100)");
  rampaBancoMotores(-100, -100, 450);
  stopMotors();
  Serial.println(">> [TEST MOTORES] Motores detenidos.");
  lastLoopUs = micros();
}

void ejecutarTestTurbina() {
  if (modo_competencia) return;
  detenerTodo();
  Serial.println(">> [TEST TURBINA] Prueba segura 25% (1250 us) por 1.5 s...");
  for (int pct = 5; pct <= 25; pct += 5) {
    setTurbina(pct);
    delay(40);
  }
  delay(1300);
  setTurbina(0);
  Serial.println(">> [TEST TURBINA] Turbina detenida (0% / 1000 us).");
  lastLoopUs = micros();
}

// -----------------------------------------------------------------------------
// PERSISTENCIA EN FLASH NVS (Preferences namespace "chavalin")
// -----------------------------------------------------------------------------
float readNvsFloatSafe(Preferences& prefs, const char* key, float def_val) {
  float v = prefs.getFloat(key, def_val);
  return (isnan(v) || isinf(v) || v < 0.0f) ? def_val : v;
}

void cargarConfigNVS() {
  PidConfig p = PITS_DEF;
  PidConfig c = COMP_DEF;

  Preferences prefs;
  if (prefs.begin(NVS_NAMESPACE, false)) {
    p.vel_recta         = constrain(prefs.getInt("p_vr",  PITS_DEF.vel_recta), 0, 255);
    p.vel_curva         = constrain(prefs.getInt("p_vc",  PITS_DEF.vel_curva), 0, 255);
    p.k_freno           = constrain(prefs.getInt("p_kf",  PITS_DEF.k_freno),   0, 255);
    p.kp_base           = readNvsFloatSafe(prefs, "p_kpb", PITS_DEF.kp_base);
    p.kp_curva          = readNvsFloatSafe(prefs, "p_kpc", PITS_DEF.kp_curva);
    p.kd_phys           = readNvsFloatSafe(prefs, "p_kd",  PITS_DEF.kd_phys);
    p.turbina_base_pct  = constrain(prefs.getInt("p_tb",  PITS_DEF.turbina_base_pct), 0, 100);
    p.turbina_curva_pct = constrain(prefs.getInt("p_tc",  PITS_DEF.turbina_curva_pct), p.turbina_base_pct, 100);

    // Migración única: iniciar con turbina desactivada (0/0%) hasta que el usuario la active en la Web
    if (prefs.getUChar("t_off1", 0) == 0) {
      p.turbina_base_pct  = 0;
      p.turbina_curva_pct = 0;
      prefs.putInt("p_tb", 0);
      prefs.putInt("p_tc", 0);
      prefs.putInt("tb", 0);
      prefs.putInt("tc", 0);
      prefs.putUChar("t_off1", 1);
    }

    c.vel_recta         = constrain(prefs.getInt("c_vr",  COMP_DEF.vel_recta), 0, 255);
    c.vel_curva         = constrain(prefs.getInt("c_vc",  COMP_DEF.vel_curva), 0, 255);
    c.k_freno           = constrain(prefs.getInt("c_kf",  COMP_DEF.k_freno),   0, 255);
    c.kp_base           = readNvsFloatSafe(prefs, "c_kpb", COMP_DEF.kp_base);
    c.kp_curva          = readNvsFloatSafe(prefs, "c_kpc", COMP_DEF.kp_curva);
    c.kd_phys           = readNvsFloatSafe(prefs, "c_kd",  COMP_DEF.kd_phys);
    c.turbina_base_pct  = constrain(prefs.getInt("c_tb",  COMP_DEF.turbina_base_pct), 20, 100);
    c.turbina_curva_pct = constrain(prefs.getInt("c_tc",  COMP_DEF.turbina_curva_pct), c.turbina_base_pct, 100);

    if (prefs.getUChar("cal_ok", 0) == 1 &&
        prefs.getBytesLength("cmin") == sizeof(cal_min_mv) &&
        prefs.getBytesLength("cmax") == sizeof(cal_max_mv)) {
      prefs.getBytes("cmin", cal_min_mv, sizeof(cal_min_mv));
      prefs.getBytes("cmax", cal_max_mv, sizeof(cal_max_mv));
      usa_barrido = true;
      calibrado_en_blanco = true;
    }
    prefs.end();
  }

  portENTER_CRITICAL(&stateMux);
  cfg_pits = p;
  cfg_comp = c;
  portEXIT_CRITICAL(&stateMux);
}

bool guardarConfigNVS() {
  PidConfig p, c;
  portENTER_CRITICAL(&stateMux);
  p = cfg_pits;
  c = cfg_comp;
  portEXIT_CRITICAL(&stateMux);

  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) return false;

  prefs.putInt("p_vr",   p.vel_recta);
  prefs.putInt("p_vc",   p.vel_curva);
  prefs.putInt("p_kf",   p.k_freno);
  prefs.putFloat("p_kpb", p.kp_base);
  prefs.putFloat("p_kpc", p.kp_curva);
  prefs.putFloat("p_kd",  p.kd_phys);
  prefs.putInt("p_tb",   p.turbina_base_pct);
  prefs.putInt("p_tc",   p.turbina_curva_pct);

  prefs.putInt("c_vr",   c.vel_recta);
  prefs.putInt("c_vc",   c.vel_curva);
  prefs.putInt("c_kf",   c.k_freno);
  prefs.putFloat("c_kpb", c.kp_base);
  prefs.putFloat("c_kpc", c.kp_curva);
  prefs.putFloat("c_kd",  c.kd_phys);
  prefs.putInt("c_tb",   c.turbina_base_pct);
  prefs.putInt("c_tc",   c.turbina_curva_pct);

  // Sincronización directa con seguidor_pro.ino (firmware oficial Cero Wi-Fi)
  int pro_tb = constrain(p.turbina_base_pct, 0, 100);
  int pro_tc = (pro_tb == 0) ? 0 : constrain(max(pro_tb, p.turbina_curva_pct), 0, 100);
  prefs.putInt("vr",    p.vel_recta);
  prefs.putInt("vc",    p.vel_curva);
  prefs.putInt("kf",    p.k_freno);
  prefs.putFloat("kpb", p.kp_base);
  prefs.putFloat("kpc", p.kp_curva);
  prefs.putFloat("kd",  p.kd_phys);
  prefs.putInt("tb",    pro_tb);
  prefs.putInt("tc",    pro_tc);

  prefs.end();
  return true;
}

// -----------------------------------------------------------------------------
// LECTURA DE SENSORES (ANTI-SATURACIÓN ADC ESP32-C3 + PESOS_V[16])
// -----------------------------------------------------------------------------
void seleccionarCanalMux(uint8_t canal, uint16_t settle_us) {
  digitalWrite(PIN_S0, (canal & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_S1, (canal & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_S2, (canal & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_S3, (canal & 0x08) ? HIGH : LOW);
  delayMicroseconds(settle_us);
}

void leerSensores(bool modo_rapido_2ms) {
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_en((gpio_num_t)PIN_SIG);

  // En 500 Hz (2 ms) usamos 12 us de asentamiento y 2 muestras (~1.15 ms total en los 16 canales)
  const uint16_t settle_us = modo_rapido_2ms ? 12 : 40;
  const int muestras = modo_rapido_2ms ? 2 : 4;

  for (uint8_t pos = 0; pos < 16; pos++) {
    uint8_t ch = canalFisico(pos);
    seleccionarCanalMux(ch, settle_us);
    uint32_t suma = 0;
    for (int m = 0; m < muestras; m++) {
      suma += analogReadMilliVolts(PIN_SIG);
    }
    lecturas_mv[pos] = (uint16_t)(suma / muestras);

    if (calibracion_barrido_activa) {
      if (lecturas_mv[pos] < cal_min_mv[pos]) cal_min_mv[pos] = lecturas_mv[pos];
      if (lecturas_mv[pos] > cal_max_mv[pos]) cal_max_mv[pos] = lecturas_mv[pos];
    }

    // En modo aire, si un canal arrancó desconectado (<60 mV) y recupera reposo, auto-ajustar base
    if (!calibracion_barrido_activa && !usa_barrido && !calibrado_en_blanco &&
        base_mv[pos] < 60 && lecturas_mv[pos] >= 2450) {
      base_mv[pos] = lecturas_mv[pos];
    }

    if (!calibracion_barrido_activa && !usa_barrido && base_mv[pos] < 60) {
      norm_s[pos] = 0;
    } else if (calibracion_barrido_activa) {
      int span = (int)cal_max_mv[pos] - (int)cal_min_mv[pos];
      int d    = (int)lecturas_mv[pos] - (int)cal_min_mv[pos];
      norm_s[pos] = (span >= 60 && d > 30) ? (uint16_t)constrain(((long)d * 1000L) / span, 0, 1000) : 0;
    } else if (usa_barrido) {
      int span = (int)cal_max_mv[pos] - (int)cal_min_mv[pos];
      if (span < 80) span = RANGO_MAX_MV;
      int delta = (int)lecturas_mv[pos] - (int)cal_min_mv[pos];
      int umbral = max(35, span / 6);
      if (delta <= umbral) {
        norm_s[pos] = 0;
      } else {
        int escala = max(umbral + 50, (span * 85) / 100);
        long n = ((long)(delta - umbral) * 1000L) / (escala - umbral);
        norm_s[pos] = (uint16_t)constrain(n, 0, 1000);
      }
    } else {
      int delta = calibrado_en_blanco
                    ? ((int)lecturas_mv[pos] - (int)base_mv[pos])
                    : ((int)base_mv[pos] - (int)lecturas_mv[pos]);
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
  uint32_t acum[16] = {0};
  for (int m = 0; m < 16; m++) {
    gpio_pullup_dis((gpio_num_t)PIN_SIG);
    gpio_pulldown_en((gpio_num_t)PIN_SIG);
    for (uint8_t pos = 0; pos < 16; pos++) {
      uint8_t ch = canalFisico(pos);
      seleccionarCanalMux(ch, 40);
      uint32_t suma = 0;
      for (int k = 0; k < 4; k++) {
        suma += analogReadMilliVolts(PIN_SIG);
      }
      acum[pos] += (suma / 4);
    }
    delay(4);
  }

  for (int i = 0; i < 16; i++) {
    base_mv[i] = (uint16_t)(acum[i] / 16);
  }

  // Mediana de los 16 sensores (I0..I15) para detectar fondo blanco vs aire
  uint16_t ord[16];
  for (int i = 0; i < 16; i++) ord[i] = base_mv[i];
  for (int i = 0; i < 15; i++) {
    for (int j = i + 1; j < 16; j++) {
      if (ord[j] < ord[i]) {
        uint16_t tmp = ord[i]; ord[i] = ord[j]; ord[j] = tmp;
      }
    }
  }

  int canales_bajos = 0;
  for (int i = 0; i < 16; i++) {
    if (base_mv[i] < 60) canales_bajos++;
  }

  // Conservamos la base individual de cada canal en ambos modos para compensar diferencias entre barras
  calibrado_en_blanco = (ord[7] < 1000);

  if (Serial.availableForWrite() >= 64) {
    Serial.printf("Modo Calibracion (16ch I0..I15): %s\n",
                  calibrado_en_blanco ? "BLANCO (detecta linea negra)" : "AIRE (detecta papel blanco)");
    Serial.print("Base mV (I0 -> I15): ");
    for (int i = 0; i < 16; i++) {
      Serial.printf("I%d:%u ", i, base_mv[i]);
    }
    Serial.println();
    if (canales_bajos == 16) {
      Serial.println("[!] ALERTA HARDWARE: Canales activos ~0 mV -> Revisar SIG (GPIO 0), EN a GND o 3.3V!");
    }
  }
  lastLoopUs = micros();
}

void iniciarCalibracionBarrido() {
  detenerTodo();
  for (int i = 0; i < 16; i++) {
    cal_min_mv[i] = 4095;
    cal_max_mv[i] = 0;
  }
  calibracion_barrido_activa = true;
  cal_fin_ms = millis() + 3500UL;
  if (Serial.availableForWrite() >= 64) {
    Serial.println(">> [CAL] Barrido 3.5s iniciado: pasa los sensores sobre BLANCO y LINEA NEGRA...");
  }
  lastLoopUs = micros();
}

void finalizarCalibracionBarrido() {
  calibracion_barrido_activa = false;
  uint16_t spans[16];
  int barridos = 0;

  for (int i = 0; i < 16; i++) {
    int sp = (int)cal_max_mv[i] - (int)cal_min_mv[i];
    if (cal_min_mv[i] >= 60 && cal_min_mv[i] < 1600 && sp >= 90) {
      spans[barridos++] = (uint16_t)sp;
    }
  }

  Preferences prefs;
  if (barridos >= 3) {
    for (int i = 0; i < barridos - 1; i++) {
      for (int j = i + 1; j < barridos; j++) {
        if (spans[j] < spans[i]) {
          uint16_t tmp = spans[i]; spans[i] = spans[j]; spans[j] = tmp;
        }
      }
    }
    uint16_t med_span = spans[barridos / 2];
    for (int i = 0; i < 16; i++) {
      if ((int)cal_max_mv[i] - (int)cal_min_mv[i] < 90) {
        cal_max_mv[i] = (uint16_t)min(4095, (int)cal_min_mv[i] + (int)med_span);
      }
    }
    usa_barrido = true;
    calibrado_en_blanco = true;
    if (prefs.begin(NVS_NAMESPACE, false)) {
      prefs.putUChar("cal_ok", 1);
      prefs.putBytes("cmin", cal_min_mv, sizeof(cal_min_mv));
      prefs.putBytes("cmax", cal_max_mv, sizeof(cal_max_mv));
      prefs.end();
    }
    if (Serial.availableForWrite() >= 48) {
      Serial.printf(">> [CAL] Barrido 16ch OK (%d/16 ch, span med=%u mV) y guardado en NVS.\n", barridos, med_span);
    }
  } else {
    usa_barrido = false;
    uint16_t ord[16];
    for (int i = 0; i < 16; i++) {
      if (cal_min_mv[i] <= cal_max_mv[i]) {
        base_mv[i] = (uint16_t)(((uint32_t)cal_min_mv[i] + (uint32_t)cal_max_mv[i]) / 2UL);
      }
      ord[i] = base_mv[i];
    }
    for (int i = 0; i < 15; i++) {
      for (int j = i + 1; j < 16; j++) {
        if (ord[j] < ord[i]) {
          uint16_t tmp = ord[i]; ord[i] = ord[j]; ord[j] = tmp;
        }
      }
    }
    calibrado_en_blanco = (ord[7] < 1000);
    if (prefs.begin(NVS_NAMESPACE, false)) {
      prefs.putUChar("cal_ok", 0);
      prefs.end();
    }
    if (Serial.availableForWrite() >= 48) {
      Serial.println(">> [CAL] Base fija actualizada (sin barrido de linea).");
    }
  }
  lastLoopUs = micros();
}

float calcularErrorLinea() {
  long suma_ponderada = 0;
  long suma_pesos = 0;
  for (int i = 0; i < 16; i++) {
    if (norm_s[i] > 0) {
      suma_ponderada += (long)PESOS_V[i] * norm_s[i];
      suma_pesos += norm_s[i];
    }
  }
  if (suma_pesos == 0) {
    if (!seguidor_activo) return 0.0f;
    if (fabsf(error_anterior) > 10.0f) {
      return (error_anterior > 0.0f) ? 750.0f : -750.0f;
    }
    return error_anterior;
  }
  return (float)suma_ponderada / (float)suma_pesos;
}

// -----------------------------------------------------------------------------
// SERVIDOR HTTP ESP-IDF (TELEMETRÍA PITS + CONFIG NVS)
// -----------------------------------------------------------------------------
static esp_err_t handleRoot(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handleData(httpd_req_t *req) {
  char json[896];
  bool run = seguidor_activo;
  bool comp = modo_competencia;
  int cd_tenths = 0;
  if (cuenta_regresiva_activa) {
    int32_t rem = (int32_t)(cuenta_fin_ms - millis());
    if (rem > 0) cd_tenths = (int)((rem + 99) / 100);
  }
  int cal_tenths = 0;
  if (calibracion_barrido_activa) {
    int32_t rem = (int32_t)(cal_fin_ms - millis());
    if (rem > 0) cal_tenths = (int)((rem + 99) / 100);
  }
  const char* cal_mode = usa_barrido ? "BARRIDO" : (calibrado_en_blanco ? "BLANCO" : "AIRE");
  int t_pct = turbina_pct;
  bool t_on = (t_pct > 0);
  float err = error_actual;
  int mL = motor_L;
  int mR = motor_R;

  PidConfig c;
  portENTER_CRITICAL(&stateMux);
  c = comp ? cfg_comp : cfg_pits;
  portEXIT_CRITICAL(&stateMux);

  int len = snprintf(json, sizeof(json),
    "{\"run\":%s,\"comp\":%s,\"cd\":%d,\"cal\":%d,\"cm\":\"%s\",\"dt\":%d,\"tOn\":%s,\"turb\":%d,\"err\":%.0f,\"mL\":%d,\"mR\":%d,"
    "\"vR\":%d,\"vC\":%d,\"kF\":%d,\"kpB\":%.3f,\"kpC\":%.3f,\"kd\":%.4f,\"tB\":%d,\"tC\":%d,"
    "\"s\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u],"
    "\"mv\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u]}",
    run ? "true" : "false",
    comp ? "true" : "false",
    cd_tenths,
    cal_tenths,
    cal_mode,
    comp ? 2 : 4,
    t_on ? "true" : "false",
    t_pct,
    err, mL, mR,
    c.vel_recta, c.vel_curva, c.k_freno,
    c.kp_base, c.kp_curva, c.kd_phys,
    c.turbina_base_pct, c.turbina_curva_pct,
    norm_s[0], norm_s[1], norm_s[2], norm_s[3],
    norm_s[4], norm_s[5], norm_s[6], norm_s[7],
    norm_s[8], norm_s[9], norm_s[10], norm_s[11],
    norm_s[12], norm_s[13], norm_s[14], norm_s[15],
    lecturas_mv[0], lecturas_mv[1], lecturas_mv[2], lecturas_mv[3],
    lecturas_mv[4], lecturas_mv[5], lecturas_mv[6], lecturas_mv[7],
    lecturas_mv[8], lecturas_mv[9], lecturas_mv[10], lecturas_mv[11],
    lecturas_mv[12], lecturas_mv[13], lecturas_mv[14], lecturas_mv[15]
  );
  if (len < 0) len = 0;
  if (len >= (int)sizeof(json)) len = (int)sizeof(json) - 1;

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, json, len);
}

static esp_err_t handleConfig(httpd_req_t *req) {
  char json[512];
  PidConfig p, c;
  portENTER_CRITICAL(&stateMux);
  p = cfg_pits;
  c = cfg_comp;
  portEXIT_CRITICAL(&stateMux);

  int len = snprintf(json, sizeof(json),
    "{\"active\":\"%s\","
    "\"pits\":{\"vr\":%d,\"vc\":%d,\"kf\":%d,\"kpb\":%.3f,\"kpc\":%.3f,\"kd\":%.4f,\"tb\":%d,\"tc\":%d},"
    "\"comp\":{\"vr\":%d,\"vc\":%d,\"kf\":%d,\"kpb\":%.3f,\"kpc\":%.3f,\"kd\":%.4f,\"tb\":%d,\"tc\":%d}}",
    modo_competencia ? "comp" : "pits",
    p.vel_recta, p.vel_curva, p.k_freno, p.kp_base, p.kp_curva, p.kd_phys, p.turbina_base_pct, p.turbina_curva_pct,
    c.vel_recta, c.vel_curva, c.k_freno, c.kp_base, c.kp_curva, c.kd_phys, c.turbina_base_pct, c.turbina_curva_pct
  );
  if (len < 0) len = 0;
  if (len >= (int)sizeof(json)) len = (int)sizeof(json) - 1;

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, json, len);
}

static esp_err_t handleCmd(httpd_req_t *req) {
  char q[64];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
    char c[24];
    if (httpd_query_key_value(q, "c", c, sizeof(c)) == ESP_OK) {
      if (strcmp(c, "start") == 0) {
        cmd_start_pendiente = true;
      } else if (strcmp(c, "start3") == 0) {
        cmd_start3_pendiente = true;
      } else if (strcmp(c, "stop") == 0) {
        cmd_stop_pendiente = true;
      } else if (strcmp(c, "turb_on") == 0) {
        cmd_turb_on_pendiente = true;
      } else if (strcmp(c, "turb_off") == 0) {
        turbina_manual = false;
        cmd_turb_off_pendiente = true;
      } else if (strcmp(c, "calibrate") == 0) {
        cmd_stop_pendiente = true;
        calibrar_pendiente = true;
      } else if (strcmp(c, "test") == 0) {
        cmd_stop_pendiente = true;
        test_motores_pendiente = true;
      }
    }
  }
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t handlePID(httpd_req_t *req) {
  char q[256], val[32];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
    if (httpd_query_key_value(q, "reset", val, sizeof(val)) == ESP_OK && atoi(val) == 1) {
      portENTER_CRITICAL(&stateMux);
      cfg_pits = PITS_DEF;
      cfg_comp = COMP_DEF;
      portEXIT_CRITICAL(&stateMux);
      guardarConfigNVS();
      if (turbina_manual && !seguidor_activo) cmd_turb_on_pendiente = true;
      httpd_resp_set_type(req, "text/plain");
      return httpd_resp_send(req, "RESET_OK", 8);
    }

    char mode[16] = "pits";
    httpd_query_key_value(q, "mode", mode, sizeof(mode));
    bool edit_comp = (strcmp(mode, "comp") == 0);

    PidConfig tmp;
    portENTER_CRITICAL(&stateMux);
    tmp = edit_comp ? cfg_comp : cfg_pits;
    portEXIT_CRITICAL(&stateMux);

    if (httpd_query_key_value(q, "vr", val, sizeof(val)) == ESP_OK ||
        httpd_query_key_value(q, "spd", val, sizeof(val)) == ESP_OK) {
      tmp.vel_recta = constrain(atoi(val), 0, 255);
    }
    if (httpd_query_key_value(q, "vc", val, sizeof(val)) == ESP_OK) {
      tmp.vel_curva = constrain(atoi(val), 0, 255);
    }
    if (httpd_query_key_value(q, "kf", val, sizeof(val)) == ESP_OK) {
      tmp.k_freno = constrain(atoi(val), 0, 255);
    }
    if (httpd_query_key_value(q, "kpb", val, sizeof(val)) == ESP_OK ||
        httpd_query_key_value(q, "kp",  val, sizeof(val)) == ESP_OK) {
      float v = strtof(val, NULL);
      if (!isnan(v) && !isinf(v)) tmp.kp_base = fmaxf(0.0f, v);
    }
    if (httpd_query_key_value(q, "kpc", val, sizeof(val)) == ESP_OK) {
      float v = strtof(val, NULL);
      if (!isnan(v) && !isinf(v)) tmp.kp_curva = fmaxf(0.0f, v);
    }
    if (httpd_query_key_value(q, "kd", val, sizeof(val)) == ESP_OK) {
      float v = strtof(val, NULL);
      if (!isnan(v) && !isinf(v)) tmp.kd_phys = fmaxf(0.0f, v);
    }
    if (httpd_query_key_value(q, "tb", val, sizeof(val)) == ESP_OK ||
        httpd_query_key_value(q, "turb", val, sizeof(val)) == ESP_OK) {
      int min_tb = edit_comp ? 20 : 0; // En competencia la turbina nunca puede arrancar en 0%
      tmp.turbina_base_pct = constrain(atoi(val), min_tb, 100);
    }
    if (httpd_query_key_value(q, "tc", val, sizeof(val)) == ESP_OK) {
      tmp.turbina_curva_pct = constrain(atoi(val), tmp.turbina_base_pct, 100);
    } else if (tmp.turbina_curva_pct < tmp.turbina_base_pct) {
      tmp.turbina_curva_pct = tmp.turbina_base_pct;
    }

    portENTER_CRITICAL(&stateMux);
    if (edit_comp) cfg_comp = tmp;
    else           cfg_pits = tmp;
    portEXIT_CRITICAL(&stateMux);

    guardarConfigNVS();

    if (!edit_comp && turbina_manual && !seguidor_activo) {
      cmd_turb_on_pendiente = true;
    }
  }
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", 2);
}

void iniciarServidorHTTP() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_open_sockets = 7;
  config.max_uri_handlers = 8;
  config.lru_purge_enable = true;
  config.recv_wait_timeout = 2;
  config.send_wait_timeout = 2;

  if (httpd_start(&server, &config) == ESP_OK) {
    static const httpd_uri_t uri_root   = { "/", HTTP_GET, handleRoot, NULL };
    static const httpd_uri_t uri_data   = { "/api/data", HTTP_GET, handleData, NULL };
    static const httpd_uri_t uri_config = { "/api/config", HTTP_GET, handleConfig, NULL };
    static const httpd_uri_t uri_cmd    = { "/api/cmd", HTTP_GET, handleCmd, NULL };
    static const httpd_uri_t uri_pid    = { "/api/pid", HTTP_GET, handlePID, NULL };
    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_data);
    httpd_register_uri_handler(server, &uri_config);
    httpd_register_uri_handler(server, &uri_cmd);
    httpd_register_uri_handler(server, &uri_pid);
  }
}

// -----------------------------------------------------------------------------
// PROCESAMIENTO DE ENTRADAS FÍSICAS (BOTÓN BOOT = 9 PARA CALIBRACIÓN)
// -----------------------------------------------------------------------------
void procesarEntradasFisicas(uint32_t nowUs) {
  (void)nowUs;

  // Botón físico BOOT (GPIO 9, INPUT_PULLUP) para recalibrar cuando está detenido
  static uint32_t boot_press_ms = 0;
  static bool boot_triggered = false;
  if (!seguidor_activo && !cuenta_regresiva_activa && !calibracion_barrido_activa) {
    if (digitalRead(PIN_BOOT_CAL) == LOW) {
      uint32_t nowMs = millis();
      if (boot_press_ms == 0) {
        boot_press_ms = nowMs;
      } else if (!boot_triggered && (uint32_t)(nowMs - boot_press_ms) >= 30UL) {
        boot_triggered = true;
        iniciarCalibracionBarrido();
      }
    } else {
      boot_press_ms = 0;
      boot_triggered = false;
    }
  } else {
    boot_press_ms = 0;
    boot_triggered = false;
  }
}

// -----------------------------------------------------------------------------
// SETUP & LOOP
// -----------------------------------------------------------------------------
void setup() {
  // Apagar motores y armar ESC ANTES de cualquier delay para minimizar el tirón al encender
  ledcAttach(PIN_MOT_A1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_A2, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_B1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_B2, PWM_FREQ, PWM_RES);
  stopMotors();

  ledcAttach(PIN_TURBINA, ESC_PWM_FREQ, ESC_PWM_RES);
  setTurbina(0);

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0); // Evita bloqueo USB CDC si nadie lee COM5
  delay(800);

  // Entradas físicas: Receptor del Juez (RRT) y Botón BOOT
  pinMode(PIN_GO, INPUT_PULLDOWN);
  pinMode(PIN_BOOT_CAL, INPUT_PULLUP);

  // Mux & IR
  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_LED_ON, OUTPUT);
  digitalWrite(PIN_LED_ON, HIGH);

  // ADC Anti-Saturación ESP32-C3 (GPIO 0)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogReadMilliVolts(PIN_SIG);
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_en((gpio_num_t)PIN_SIG);

  // Calibración inicial de referencia y luego carga de NVS (mantiene barrido guardado si existe)
  calibrarBase();
  cargarConfigNVS();

  // Iniciar Wi-Fi Access Point en Pits
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  delay(100);
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);
  bool ap_ok = WiFi.softAP(AP_SSID, AP_PASS, 8);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.setSleep(false);
  IPAddress ip = WiFi.softAPIP();
  delay(250);

  iniciarServidorHTTP();

  Serial.println("\n=======================================================");
  Serial.println("  SEGUIDOR PRO v1 - TELEMETRÍA WEB + NVS + COMPETENCIA");
  Serial.println("=======================================================");
  Serial.printf("  AP Iniciado: %s | TX Power: %d\n", ap_ok ? "OK" : "ERROR", (int)WiFi.getTxPower());
  Serial.printf("  Wi-Fi: %s | Pass: %s | http://%s\n", AP_SSID, AP_PASS, ip.toString().c_str());
  Serial.printf("  NVS Pits 12ms -> vR:%d vC:%d kF:%d KpB:%.2f KpC:%.2f Kd:%.3f Turb:%d/%d%%\n",
                cfg_pits.vel_recta, cfg_pits.vel_curva, cfg_pits.k_freno,
                cfg_pits.kp_base, cfg_pits.kp_curva, cfg_pits.kd_phys,
                cfg_pits.turbina_base_pct, cfg_pits.turbina_curva_pct);
  Serial.printf("  NVS Comp  2ms -> vR:%d vC:%d kF:%d KpB:%.2f KpC:%.2f Kd:%.3f Turb:%d/%d%%\n",
                cfg_comp.vel_recta, cfg_comp.vel_curva, cfg_comp.k_freno,
                cfg_comp.kp_base, cfg_comp.kp_curva, cfg_comp.kd_phys,
                cfg_comp.turbina_base_pct, cfg_comp.turbina_curva_pct);
  Serial.println("=======================================================\n");

  lastLoopUs = micros();
}

void loop() {
  uint32_t nowUs = micros();

  // 1. Verificar botón físico BOOT (GPIO 21 PIN_GO deshabilitado en modo Web)
  procesarEntradasFisicas(nowUs);

  // Freno en 2 fases anti-pico: 50 ms coast (0,0) -> 150 ms freno activo -> stop
  if (freno_fase == 1 && (int32_t)(millis() - freno_fin_ms) >= 0) {
    brakeMotors();
    freno_fase = 2;
    freno_fin_ms = millis() + 150UL;
  } else if (freno_fase == 2 && (int32_t)(millis() - freno_fin_ms) >= 0) {
    freno_fase = 0;
    stopMotors();
  }

  // 2. Despachar comandos pendientes de HTTP de forma segura en loop()
  if (cmd_stop_pendiente) {
    cmd_stop_pendiente = false;
    detenerConFreno();
  }
  if (cmd_turb_off_pendiente) {
    cmd_turb_off_pendiente = false;
    turbina_manual = false;
    if (!seguidor_activo && !cuenta_regresiva_activa) setTurbina(0);
  }
  if (cmd_turb_on_pendiente) {
    cmd_turb_on_pendiente = false;
    if (!seguidor_activo && !cuenta_regresiva_activa && !calibracion_barrido_activa) {
      turbina_manual = true;
      PidConfig p;
      portENTER_CRITICAL(&stateMux);
      p = cfg_pits;
      portEXIT_CRITICAL(&stateMux);
      setTurbina(p.turbina_base_pct);
    }
  }
  if (calibrar_pendiente) {
    calibrar_pendiente = false;
    iniciarCalibracionBarrido();
  }
  if (test_motores_pendiente) {
    test_motores_pendiente = false;
    if (!calibracion_barrido_activa) {
      ejecutarTestMotores();
    }
  }
  if (cmd_start3_pendiente) {
    cmd_start3_pendiente = false;
    if (!seguidor_activo && !calibracion_barrido_activa) {
      freno_fase = 0;
      stopMotors();
      turbina_manual = false;
      turb_cmd = 0.0f;
      setTurbina(0);
      cuenta_regresiva_activa = true;
      cuenta_fin_ms = millis() + 3000UL;
    }
  }
  if (cmd_start_pendiente) {
    cmd_start_pendiente = false;
    if (!modo_competencia && !calibracion_barrido_activa) {
      iniciarCarrera(false); // Arranque desde UI web en modo Pits 12 ms
    }
  }

  // Finalizar barrido de calibración de 3.5 s si venció el temporizador
  if (calibracion_barrido_activa && (int32_t)(millis() - cal_fin_ms) >= 0) {
    finalizarCalibracionBarrido();
  }

  // Cuenta regresiva de 3 s con pre-arranque de turbina en los últimos 500 ms
  if (cuenta_regresiva_activa) {
    int32_t rem_ms = (int32_t)(cuenta_fin_ms - millis());
    if (rem_ms <= 0) {
      cuenta_regresiva_activa = false;
      iniciarCarrera(false);
    } else if (rem_ms <= 500) {
      PidConfig p;
      portENTER_CRITICAL(&stateMux);
      p = cfg_pits;
      portEXIT_CRITICAL(&stateMux);
      float frac = (float)(500 - rem_ms) / 500.0f;
      turb_cmd = (float)p.turbina_base_pct * frac;
      setTurbinaFloat(turb_cmd);
    }
  }

  // 3. Lazo temporizado PID Dinámico: 2 ms (500 Hz, Wi-Fi OFF), 5 ms (barrido) o 12 ms (~83 Hz, Wi-Fi ON)
  nowUs = micros();
  uint32_t periodUs = modo_competencia ? LOOP_US_COMP : (calibracion_barrido_activa ? 5000UL : LOOP_US_PITS);
  if ((uint32_t)(nowUs - lastLoopUs) >= periodUs) {
    if ((uint32_t)(nowUs - lastLoopUs) >= periodUs * 2UL) {
      lastLoopUs = nowUs;
    } else {
      lastLoopUs += periodUs;
    }

    leerSensores(modo_competencia || seguidor_activo);
    error_actual = calcularErrorLinea();

    if (seguidor_activo) {
      // Protección: si 13+ sensores leen alto (>650) por 250 ms (robot levantado/fuera de pista), cortar
      int cnt_altos = 0;
      for (int i = 0; i < 16; i++) {
        if (norm_s[i] > 650) cnt_altos++;
      }
      if (cnt_altos >= 13) {
        if (todo_oscuro_ms == 0) {
          todo_oscuro_ms = millis();
        } else if ((uint32_t)(millis() - todo_oscuro_ms) >= 250UL) {
          detenerTodo();
          if (Serial.availableForWrite() >= 64) {
            Serial.println(">> [PROTECCION] Robot levantado / fuera de pista -> Paro seguro.");
          }
        }
      } else {
        todo_oscuro_ms = 0;
      }

      if (seguidor_activo) {
        PidConfig cfg;
        portENTER_CRITICAL(&stateMux);
        cfg = modo_competencia ? cfg_comp : cfg_pits;
        portEXIT_CRITICAL(&stateMux);

        float dt_s = modo_competencia ? DT_COMP_S : DT_PITS_S;
        float u_err = fminf(1.0f, fabsf(error_actual) / 750.0f);
        float kp_din = cfg.kp_base + cfg.kp_curva * (u_err * u_err);

        float alpha_d = dt_s / (TAU_D + dt_s);
        float de_dt = (error_actual - error_anterior) / dt_s;
        d_filt = alpha_d * de_dt + (1.0f - alpha_d) * d_filt;

        float pid_cap     = fminf(MAX_PID, (float)cfg.vel_recta * 1.15f + 20.0f);
        float salida_pid  = constrain(kp_din * error_actual + cfg.kd_phys * d_filt, -pid_cap, pid_cap);

        float alpha_slow  = dt_s / (TAU_SLOW + dt_s);
        u_slow += alpha_slow * (u_err - u_slow);
        float u_jump      = fmaxf(0.0f, u_err - u_slow);
        float brake_floor = -fminf(85.0f, (float)cfg.vel_recta * 0.65f);
        float vel_base    = fmaxf((float)cfg.vel_curva, (float)cfg.vel_recta - (float)cfg.k_freno * u_err);
        float vel_din     = fmaxf(brake_floor, vel_base - cfg.k_freno_dyn * (float)cfg.k_freno * u_jump);

        mot_rampa = fminf(1.0f, mot_rampa + dt_s / MOT_RAMPA_S);
        int pwm_max_fwd = min(230, max(cfg.vel_recta + 35, cfg.vel_curva + 35));
        int pwm_min_rev = -(int)lroundf(fminf(85.0f, (float)cfg.vel_recta * 0.70f));

        float tgt_A = constrain((vel_din + salida_pid) * mot_rampa, (float)pwm_min_rev, (float)pwm_max_fwd);
        float tgt_B = constrain((vel_din - salida_pid) * mot_rampa, (float)pwm_min_rev, (float)pwm_max_fwd);

        mot_cmd_L = slewUnMotor(mot_cmd_L, tgt_A, dt_s);
        mot_cmd_R = slewUnMotor(mot_cmd_R, tgt_B, dt_s);
        setMotors((int)lroundf(mot_cmd_L), (int)lroundf(mot_cmd_R));

        int t_base = cfg.turbina_base_pct;
        if (modo_competencia && t_base < 20) t_base = COMP_DEF.turbina_base_pct;
        int t_curva = (t_base == 0 && !modo_competencia) ? 0 : max(t_base, cfg.turbina_curva_pct);

        float turb_tgt = (float)t_base + (float)(t_curva - t_base) * u_err;
        if (!turbina_rampa_done) {
          turb_cmd += (float)max(t_base, 1) * (dt_s / TURB_RAMPA_S);
          if (turb_cmd >= (float)t_base) {
            turb_cmd = (float)t_base;
            turbina_rampa_done = true;
          }
        } else if (turb_tgt >= turb_cmd) {
          turb_cmd = fminf(turb_tgt, turb_cmd + TURB_SLEW_PCT_S * dt_s);
        } else {
          turb_cmd = turb_tgt + (turb_cmd - turb_tgt) * expf(-dt_s / TAU_TURB_DECAY);
        }
        setTurbinaFloat(turb_cmd);
      }
    }

    error_anterior = error_actual;
  }

  // 4. Comandos rápidos por USB Serial ('s' paro, 'c' calibrar, 'u' test turbina 25%, 'm' test motores)
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 's' || ch == 'S') {
      detenerConFreno();
    } else if ((ch == 'c' || ch == 'C') && !seguidor_activo) {
      iniciarCalibracionBarrido();
    } else if ((ch == 'u' || ch == 'U') && !seguidor_activo && !modo_competencia) {
      ejecutarTestTurbina();
    } else if ((ch == 'm' || ch == 'M') && !seguidor_activo && !modo_competencia) {
      ejecutarTestMotores();
    }
  }

  // 5. Telemetría periódica por USB Serial solo en reposo (no bloqueante, ~6.7 Hz)
  static unsigned long lastSerialMs = 0;
  unsigned long nowMs = millis();
  if (!seguidor_activo && (uint32_t)(nowMs - lastSerialMs) >= 150UL) {
    lastSerialMs = nowMs;
    if (Serial.availableForWrite() >= 100) {
      char sbuf[112];
      int n = snprintf(sbuf, sizeof(sbuf),
        "mV (I0..I15): %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u | Err:%.0f\n",
        lecturas_mv[0], lecturas_mv[1], lecturas_mv[2], lecturas_mv[3],
        lecturas_mv[4], lecturas_mv[5], lecturas_mv[6], lecturas_mv[7],
        lecturas_mv[8], lecturas_mv[9], lecturas_mv[10], lecturas_mv[11],
        lecturas_mv[12], lecturas_mv[13], lecturas_mv[14], lecturas_mv[15],
        error_actual
      );
      if (n > 0) Serial.write((const uint8_t*)sbuf, min(n, (int)sizeof(sbuf) - 1));
    }
  }

  // En reposo cedemos 1 ms a Wi-Fi/RTOS; corriendo no bloqueamos el lazo de 4 ms / 2 ms
  if (!seguidor_activo) {
    delay(1);
  }
}
