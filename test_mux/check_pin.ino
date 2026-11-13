/*
  Test rápido de impedancia / flotación en GPIO 0 (COM)
*/
#define PIN_SIG 0

void setup() {
  Serial.begin(115200);
  delay(1500);
  analogReadResolution(12);

  // 1. Lectura normal
  pinMode(PIN_SIG, INPUT);
  delay(100);
  int v_normal = analogReadMilliVolts(PIN_SIG);

  // 2. Con PULLDOWN
  pinMode(PIN_SIG, INPUT_PULLDOWN);
  delay(100);
  int v_pulldown = analogReadMilliVolts(PIN_SIG);

  // 3. Con PULLUP
  pinMode(PIN_SIG, INPUT_PULLUP);
  delay(100);
  int v_pullup = analogReadMilliVolts(PIN_SIG);

  Serial.println("\n--- TEST DE ESTADO DEL PIN COM (GPIO 0) ---");
  Serial.printf("Sin Pull:    %d mV\n", v_normal);
  Serial.printf("Con Pulldown: %d mV\n", v_pulldown);
  Serial.printf("Con Pullup:   %d mV\n", v_pullup);
  if (v_pulldown < 200 && v_pullup > 2800) {
    Serial.println("DIAGNOSTICO: El pin GPIO 0 ESTA FLOTANDO (Desconectado o Mux en alta impedancia)");
  } else {
    Serial.println("DIAGNOSTICO: El pin GPIO 0 TIENE CARGA EXTERNA conectada.");
  }
}

void loop() {
  delay(1000);
}
