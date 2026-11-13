#include <Arduino.h>
#include "driver/gpio.h"

#define PIN_SIG      0
#define PIN_S0       6
#define PIN_S1       7
#define PIN_S2      10
#define PIN_S3      20
#define PIN_LED_ON   4

const uint8_t CTRL_PINS[5] = {PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_LED_ON};
const char*   CTRL_NAMES[5] = {"S0(GPIO6)", "S1(GPIO7)", "S2(GPIO10)", "S3(GPIO20)", "LED_ON(GPIO4)"};

uint16_t base_mv[16];
bool base_lista = false;

void setMux(uint8_t ch) {
  digitalWrite(PIN_S0, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_S1, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_S2, (ch & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_S3, (ch & 0x08) ? HIGH : LOW);
  delayMicroseconds(40);
}

void checarPinesControl() {
  Serial.println("\n[1] CHEQUEO ELECTRICO DE PINES DE CONTROL (S0..S3 y LED_ON):");
  for (int i = 0; i < 5; i++) {
    pinMode(CTRL_PINS[i], INPUT_PULLDOWN);
    delay(2);
    int v_pd = digitalRead(CTRL_PINS[i]);

    pinMode(CTRL_PINS[i], INPUT_PULLUP);
    delay(2);
    int v_pu = digitalRead(CTRL_PINS[i]);

    const char* estado = "OK (libre/conmutable)";
    if (v_pd == 1 && v_pu == 1) estado = "ATASCADO EN HIGH (3.3V)!";
    else if (v_pd == 0 && v_pu == 0) estado = "ATASCADO EN LOW (GND)!";
    Serial.printf("  %-14s -> PullDown:%d  PullUp:%d  => %s\n", CTRL_NAMES[i], v_pd, v_pu, estado);
  }

  // Checar cortos entre pines S0..S3 y LED_ON
  bool hay_corto = false;
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) pinMode(CTRL_PINS[j], INPUT_PULLDOWN);
    pinMode(CTRL_PINS[i], OUTPUT);
    digitalWrite(CTRL_PINS[i], HIGH);
    delay(2);
    for (int j = 0; j < 5; j++) {
      if (i == j) continue;
      if (digitalRead(CTRL_PINS[j]) == HIGH) {
        Serial.printf("  [!] POSIBLE CORTO entre %s y %s!\n", CTRL_NAMES[i], CTRL_NAMES[j]);
        hay_corto = true;
      }
    }
  }
  if (!hay_corto) {
    Serial.println("  Cortos entre S0..S3 / LED_ON: Ninguno detectado.");
  }

  // Restaurar salidas
  for (int i = 0; i < 5; i++) pinMode(CTRL_PINS[i], OUTPUT);
  digitalWrite(PIN_LED_ON, HIGH);
}

void checarPinSIG() {
  Serial.println("\n[2] CHEQUEO DE ENTRADA SIG / COM (GPIO 0):");
  setMux(15); // Probar en I15
  pinMode(PIN_SIG, INPUT_PULLDOWN);
  delay(5);
  int mv_pd = analogReadMilliVolts(PIN_SIG);

  pinMode(PIN_SIG, INPUT_PULLUP);
  delay(5);
  int mv_pu = analogReadMilliVolts(PIN_SIG);

  pinMode(PIN_SIG, INPUT);
  delay(5);
  int mv_in = analogReadMilliVolts(PIN_SIG);

  Serial.printf("  GPIO0 -> SinPull:%d mV | PullDown:%d mV | PullUp:%d mV\n", mv_in, mv_pd, mv_pu);

  // Dejar ADC + Hardware PullDown configurado correctamente
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogReadMilliVolts(PIN_SIG);
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_en((gpio_num_t)PIN_SIG);
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println("\n==============================================================");
  Serial.println("  CHEQUEO COMPLETO DE SENSORES, MUX CD74HC4067 Y GPIO (ESP32-C3)");
  Serial.println("==============================================================");

  checarPinesControl();
  checarPinSIG();
}

void loop() {
  // 1. Sin Pull-Down ni Pull-Up
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_dis((gpio_num_t)PIN_SIG);
  digitalWrite(PIN_LED_ON, HIGH);
  delay(5);
  Serial.print("\n1. SIN PULL (I15..I0)     : ");
  for (int pos = 0; pos < 16; pos++) {
    int ch = 15 - pos;
    setMux(ch);
    delayMicroseconds(100);
    Serial.printf("%4d ", analogReadMilliVolts(PIN_SIG));
  }
  Serial.println();

  // 2. Con Pull-Down hardware + IR ON
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_en((gpio_num_t)PIN_SIG);
  digitalWrite(PIN_LED_ON, HIGH);
  delay(5);
  uint16_t mv_on[16];
  uint16_t min_v = 9999, max_v = 0;
  Serial.print("2. PULLDOWN+IR ON(I15..I0): ");
  for (int pos = 0; pos < 16; pos++) {
    int ch = 15 - pos;
    setMux(ch);
    delayMicroseconds(100);
    uint32_t s = 0;
    for (int k = 0; k < 4; k++) s += analogReadMilliVolts(PIN_SIG);
    mv_on[pos] = s / 4;
    if (mv_on[pos] < min_v) min_v = mv_on[pos];
    if (mv_on[pos] > max_v) max_v = mv_on[pos];
    Serial.printf("%4d ", mv_on[pos]);
  }
  Serial.println();

  // 3. Con Pull-Down hardware + IR OFF
  digitalWrite(PIN_LED_ON, LOW);
  delay(5);
  Serial.print("3. PULLDOWN+IR OFF        : ");
  for (int pos = 0; pos < 16; pos++) {
    int ch = 15 - pos;
    setMux(ch);
    delayMicroseconds(100);
    Serial.printf("%4d ", analogReadMilliVolts(PIN_SIG));
  }
  Serial.println();
  digitalWrite(PIN_LED_ON, HIGH);

  if (!base_lista) {
    for (int i = 0; i < 16; i++) base_mv[i] = mv_on[i];
    base_lista = true;
  }

  // Diagnostico automatico de este ciclo
  int conectados = 0;
  int activos = 0;
  Serial.print("4. CAMBIO vs REPOSO       : ");
  for (int pos = 0; pos < 16; pos++) {
    int diff = (int)mv_on[pos] - (int)base_mv[pos];
    Serial.printf("%+4d ", diff);
    if (mv_on[pos] > 500) conectados++;
    if (abs(diff) >= 35 || mv_on[pos] < 2200) activos++;
  }
  Serial.println();
  Serial.printf("RESUMEN: Canales >500mV=%d/16 (min=%u, max=%u mV) | Con reflejo/cambio=%d/16\n",
                conectados, min_v, max_v, activos);

  if (conectados == 0) {
    Serial.println("[!] ALERTA: Todos <500mV -> Revisar cable SIG (GPIO0), EN a GND o 3.3V en barras/mux.");
  }
  delay(800);
}
