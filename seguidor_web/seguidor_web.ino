/*
  =============================================================================
  PROYECTO: SEGUIDOR DE LINEA PRO v1 - TELEMETRÍA WEB WIFI + PID + TB6612
  =============================================================================
  - Levanta una red Wi-Fi propia (Access Point) desde el ESP32-C3:
      Red (SSID): SeguidorPro-WiFi
      Contraseña: 12345678
      URL Web:    http://192.168.4.1
  - Muestra en tiempo real:
      * Gráfica de barras normalizada de los 16 sensores (0-1000)
      * Tacómetros de Motor Izquierdo y Motor Derecho (-255 a +255)
      * Osciloscopio temporal de comandos y error
      * Ajuste en caliente de Kp, Ki, Kd y Velocidad Base
  =============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include "driver/gpio.h"
#include "web_ui.h"

// --- CONFIGURACIÓN WIFI ACCESS POINT ---
const char* AP_SSID = "SeguidorPro-WiFi";
const char* AP_PASS = "12345678";

httpd_handle_t server = NULL;

// --- PINES DRIVER TB6612 (PWM 20 kHz) ---
#define PIN_MOT_A1   1   // GPIO 1: AI1 (Motor Izq)
#define PIN_MOT_A2   2   // GPIO 2: AI2 (Motor Izq)
#define PIN_MOT_B1   3   // GPIO 3: BI1 (Motor Der)
#define PIN_MOT_B2   8   // GPIO 8: BI2 (Motor Der - comparte LED azul)

const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 8;

// --- PINES MULTIPLEXOR Y SENSORES ---
#define PIN_SIG      0   // GPIO 0: ADC COM/SIG
#define PIN_S0       6   // GPIO 6: S0
#define PIN_S1       7   // GPIO 7: S1
#define PIN_S2      10   // GPIO 10: S2
#define PIN_S3      20   // GPIO 20: S3
#define PIN_LED_ON   4   // GPIO 4: Emisores IR

// --- VARIABLES DE SENSORES Y CALIBRACIÓN ---
uint16_t lecturas_mv[16]; // Orden lógico Left->Right: índice 0 = I0 (Izq) .. 15 = I15 (Der)
uint16_t base_mv[16];
uint16_t norm_s[16];      // Normalizado 0 a 1000 para la UI web
const int RANGO_MAX_MV = 450; // Rango de variación (mV) para escalar 0-1000 con alta respuesta visual
const int UMBRAL_DELTA = 25;  // Con promedio x4 el ruido es ~6 mV; 25 mV da buena sensibilidad sin falsos disparos

inline uint8_t canalFisico(uint8_t pos_izq_der) {
  return pos_izq_der; // Extremo izq (pos 0) = I0 (CH00); extremo der (pos 15) = I15 (CH15)
}

// --- VARIABLES DE CONTROL PID Y MOTORES ---
volatile bool seguidor_activo = false;
volatile bool test_motores_pendiente = false;
volatile bool calibrar_pendiente = false;

int velocidad_base = 120;
float Kp = 0.35f;
float Ki = 0.00f;
float Kd = 0.15f;

float error_actual = 0.0f;
float error_anterior = 0.0f;
float integral = 0.0f;
int motor_L = 0;
int motor_R = 0;

// -----------------------------------------------------------------------------
// CONTROL DE MOTORES TB6612
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

void setMotors(int sL, int sR) {
  setMotorA(sL);
  setMotorB(sR);
}

void stopMotors() {
  setMotors(0, 0);
  integral = 0;
  error_anterior = 0;
}

void ejecutarTestMotores() {
  stopMotors();
  setMotors(110, 0);  delay(600);
  setMotors(0, 110);  delay(600);
  setMotors(100, 100); delay(600);
  setMotors(-100, -100); delay(600);
  stopMotors();
}

// -----------------------------------------------------------------------------
// LECTURA DE SENSORES (ANTI-SATURACIÓN ADC ESP32-C3)
// -----------------------------------------------------------------------------
void seleccionarCanalMux(uint8_t canal) {
  digitalWrite(PIN_S0, (canal & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_S1, (canal & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_S2, (canal & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_S3, (canal & 0x08) ? HIGH : LOW);
  delayMicroseconds(20);
}

void leerSensores() {
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_en((gpio_num_t)PIN_SIG);

  for (uint8_t pos = 0; pos < 16; pos++) {
    uint8_t ch = canalFisico(pos);
    seleccionarCanalMux(ch);
    delayMicroseconds(60);
    uint32_t suma = 0;
    for (int m = 0; m < 4; m++) {
      suma += analogReadMilliVolts(PIN_SIG);
    }
    lecturas_mv[pos] = suma / 4;

    // Si el canal arrancó desconectado (<300 mV) y ya entró en reposo normal (>=2450 mV), auto-ajustar base
    if (base_mv[pos] < 300 && lecturas_mv[pos] >= 2450) {
      base_mv[pos] = lecturas_mv[pos];
    }

    int delta = abs((int)lecturas_mv[pos] - (int)base_mv[pos]);
    if (delta < UMBRAL_DELTA) {
      norm_s[pos] = 0;
    } else {
      long n = ((long)delta * 1000L) / RANGO_MAX_MV;
      norm_s[pos] = (uint16_t)constrain(n, 0, 1000);
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
      seleccionarCanalMux(ch);
      delayMicroseconds(60);
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
    base_mv[i] = acum[i] / 16;
    ord[i] = base_mv[i];
  }
  // Ordenar para obtener el percentil 75 (referencia de blanco en reposo)
  for (int i = 0; i < 15; i++) {
    for (int j = i + 1; j < 16; j++) {
      if (ord[j] < ord[i]) {
        uint16_t tmp = ord[i]; ord[i] = ord[j]; ord[j] = tmp;
      }
    }
  }
  uint16_t ref_blanco = ord[12];

  Serial.print("Base mV (I0 -> I15): ");
  int canales_bajos = 0;
  for (int i = 0; i < 16; i++) {
    // Si la barra está alimentada (ref_blanco > 2200) y algún sensor estaba pisando línea negra al calibrar, usar ref_blanco
    if (ref_blanco > 2200 && base_mv[i] > 300 && base_mv[i] < (ref_blanco - 60)) {
      base_mv[i] = ref_blanco;
    }
    if (base_mv[i] < 150) canales_bajos++;
    Serial.printf("I%d:%u ", i, base_mv[i]);
  }
  Serial.println();
  if (canales_bajos == 16) {
    Serial.println("[!] ALERTA HARDWARE: Todos los canales leen ~0 mV -> Revisar cable SIG (GPIO 0), EN a GND o 3.3V de Mux/Barras!");
  }
}

float calcularErrorLinea() {
  long suma_ponderada = 0;
  long suma_pesos = 0;
  for (int i = 0; i < 16; i++) {
    if (norm_s[i] > 0) {
      int pos = (i - 7) * 100 - 50; // -750 (izq I0) a +750 (der I15)
      suma_ponderada += (long)pos * norm_s[i];
      suma_pesos += norm_s[i];
    }
  }
  if (suma_pesos == 0) return seguidor_activo ? error_anterior : 0.0f;
  return (float)suma_ponderada / suma_pesos;
}

// -----------------------------------------------------------------------------
// SERVIDOR HTTP ESP-IDF CON KEEP-ALIVE Y PURGA LRU DE SOCKETS
// -----------------------------------------------------------------------------
static esp_err_t handleRoot(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handleData(httpd_req_t *req) {
  char json[420];
  int len = snprintf(json, sizeof(json),
    "{\"run\":%s,\"err\":%.0f,\"mL\":%d,\"mR\":%d,"
    "\"s\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u],"
    "\"mv\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u]}",
    seguidor_activo ? "true" : "false",
    error_actual, motor_L, motor_R,
    norm_s[0], norm_s[1], norm_s[2], norm_s[3],
    norm_s[4], norm_s[5], norm_s[6], norm_s[7],
    norm_s[8], norm_s[9], norm_s[10], norm_s[11],
    norm_s[12], norm_s[13], norm_s[14], norm_s[15],
    lecturas_mv[0], lecturas_mv[1], lecturas_mv[2], lecturas_mv[3],
    lecturas_mv[4], lecturas_mv[5], lecturas_mv[6], lecturas_mv[7],
    lecturas_mv[8], lecturas_mv[9], lecturas_mv[10], lecturas_mv[11],
    lecturas_mv[12], lecturas_mv[13], lecturas_mv[14], lecturas_mv[15]
  );
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
        seguidor_activo = true;
        integral = 0;
      } else if (strcmp(c, "stop") == 0) {
        seguidor_activo = false;
        stopMotors();
      } else if (strcmp(c, "calibrate") == 0) {
        seguidor_activo = false;
        stopMotors();
        calibrar_pendiente = true;
      } else if (strcmp(c, "test") == 0) {
        seguidor_activo = false;
        test_motores_pendiente = true;
      }
    }
  }
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t handlePID(httpd_req_t *req) {
  char q[96], val[24];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
    if (httpd_query_key_value(q, "spd", val, sizeof(val)) == ESP_OK) velocidad_base = constrain(atoi(val), 0, 255);
    if (httpd_query_key_value(q, "kp",  val, sizeof(val)) == ESP_OK) Kp = atof(val);
    if (httpd_query_key_value(q, "ki",  val, sizeof(val)) == ESP_OK) Ki = atof(val);
    if (httpd_query_key_value(q, "kd",  val, sizeof(val)) == ESP_OK) Kd = atof(val);
  }
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", 2);
}

void iniciarServidorHTTP() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_open_sockets = 7;
  config.lru_purge_enable = true;
  config.recv_wait_timeout = 2;
  config.send_wait_timeout = 2;

  if (httpd_start(&server, &config) == ESP_OK) {
    static const httpd_uri_t uri_root = { "/", HTTP_GET, handleRoot, NULL };
    static const httpd_uri_t uri_data = { "/api/data", HTTP_GET, handleData, NULL };
    static const httpd_uri_t uri_cmd  = { "/api/cmd", HTTP_GET, handleCmd, NULL };
    static const httpd_uri_t uri_pid  = { "/api/pid", HTTP_GET, handlePID, NULL };
    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_data);
    httpd_register_uri_handler(server, &uri_cmd);
    httpd_register_uri_handler(server, &uri_pid);
  }
}

// -----------------------------------------------------------------------------
// SETUP & LOOP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0); // Evita bloqueo de 1.8s en loop() cuando nadie lee COM5 por USB CDC
  delay(800);

  // Motores
  ledcAttach(PIN_MOT_A1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_A2, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_B1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_B2, PWM_FREQ, PWM_RES);
  stopMotors();

  // Mux & IR
  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_LED_ON, OUTPUT);
  digitalWrite(PIN_LED_ON, HIGH);

  // ADC Anti-Saturación ESP32-C3
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogReadMilliVolts(PIN_SIG);
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_en((gpio_num_t)PIN_SIG);

  // Iniciar Wi-Fi Access Point (fix ESP32-C3 SuperMini + 802.11b/g estable sin pérdidas de paquetes)
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  delay(100);
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);
  bool ap_ok = WiFi.softAP(AP_SSID, AP_PASS, 8);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.setSleep(false);
  IPAddress ip = WiFi.softAPIP();
  delay(250);

  calibrarBase();

  iniciarServidorHTTP();

  Serial.println("\n=======================================================");
  Serial.println("  SEGUIDOR PRO v1 - SERVIDOR WEB DE TELEMETRÍA ACTIVO");
  Serial.println("=======================================================");
  Serial.printf("  AP Iniciado: %s | TX Power: %d\n", ap_ok ? "OK" : "ERROR", (int)WiFi.getTxPower());
  Serial.printf("  1. Conéctate al Wi-Fi: %s\n", AP_SSID);
  Serial.printf("  2. Contraseña:         %s\n", AP_PASS);
  Serial.printf("  3. Abre en navegador:  http://%s\n", ip.toString().c_str());
  Serial.println("=======================================================\n");
}

void loop() {
  if (calibrar_pendiente) {
    calibrar_pendiente = false;
    calibrarBase();
  }

  if (test_motores_pendiente) {
    test_motores_pendiente = false;
    ejecutarTestMotores();
  }

  // Muestreo de sensores y PID cada 12 ms (~83 Hz) para dejar libre la CPU y radio Wi-Fi
  static unsigned long lastSensorMs = 0;
  unsigned long now = millis();
  if (now - lastSensorMs >= 12) {
    lastSensorMs = now;
    leerSensores();
    error_actual = calcularErrorLinea();

    if (seguidor_activo) {
      integral = constrain(integral + error_actual, -1000.0f, 1000.0f);
      float derivativo = error_actual - error_anterior;
      float salida_pid = (Kp * error_actual) + (Ki * integral) + (Kd * derivativo);

      int vel_A = (int)(velocidad_base + salida_pid);
      int vel_B = (int)(velocidad_base - salida_pid);
      setMotors(vel_A, vel_B);
    }
    error_anterior = error_actual;
  }

  // Paro rápido de seguridad por USB Serial si envían 's'
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 's') {
      seguidor_activo = false;
      stopMotors();
    }
  }

  // Telemetria rapida por USB Serial cada 1s en modo manual (no bloqueante si nadie lee COM5)
  static unsigned long lastSerialMs = 0;
  if (!seguidor_activo && now - lastSerialMs >= 1000) {
    lastSerialMs = now;
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
      Serial.write((const uint8_t*)sbuf, n);
    }
  }

  delay(1);
}
