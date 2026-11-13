/*
  =============================================================================
  TEST EN VIVO Y CONTINUO DE SENSORES Y MULTIPLEXOR
  =============================================================================
  CÓMO FUNCIONA EL SENSOR QTR-8A:
  - Sin nada enfrente (al aire libre o sobre negro): 
      El fototransistor no recibe luz -> Lectura ALTA (~2900 - 3300 mV / ADC ~4000).
  - Con papel blanco o tu dedo A 3-5 MILÍMETROS del sensor:
      La luz IR rebota y entra al fototransistor -> Lectura BAJA (~200 - 1500 mV).
  
  Este código mantiene encendidos los emisores IR (GPIO 4 = HIGH) y detecta
  en tiempo real cuándo acercas un papel blanco a cada sensor.
  =============================================================================
*/

#define PIN_SIG      0   // GPIO0 (COM)
#define PIN_S0       6   // S0
#define PIN_S1       7   // S1
#define PIN_S2      10   // S2
#define PIN_S3      20   // S3
#define PIN_LED_ON   4   // LED_ON

uint16_t valores_adc[16];
uint16_t valores_mv[16];

void setMux(uint8_t ch) {
  digitalWrite(PIN_S0, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_S1, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_S2, (ch & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_S3, (ch & 0x08) ? HIGH : LOW);
  delayMicroseconds(20);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  
  pinMode(PIN_LED_ON, OUTPUT);
  digitalWrite(PIN_LED_ON, HIGH); // IR SIEMPRE ENCENDIDO

  analogReadResolution(12);
  #if defined(ESP32)
    analogSetAttenuation(ADC_11db);
  #endif

  Serial.println("\n=======================================================");
  Serial.println("  PRUEBA EN VIVO: ACERCA UN PAPEL BLANCO A LOS SENSORES");
  Serial.println("=======================================================");
  Serial.println("Coloca un papel blanco a unos 3 a 5 mm de la barra.");
  Serial.println("El valor deberia caer de ~2900 mV a menos de 1800 mV.");
  Serial.println("-------------------------------------------------------\n");
}

void loop() {
  String detectados = "";

  for (uint8_t ch = 0; ch < 16; ch++) {
    setMux(ch);
    valores_adc[ch] = analogRead(PIN_SIG);
    valores_mv[ch] = analogReadMilliVolts(PIN_SIG);

    // Si el valor baja significativamente de 2600 mV, hay reflexión (blanco/dedo cerca)
    if (valores_mv[ch] < 2400) {
      detectados += " [CH" + String(ch) + ": " + String(valores_mv[ch]) + "mV]";
    }
  }

  // Imprimir resumen compacto
  Serial.print("CH 0-7 (Barra 1): ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("%4d ", valores_mv[i]);
  }
  Serial.print("| CH 8-15 (Barra 2): ");
  for (int i = 8; i < 16; i++) {
    Serial.printf("%4d ", valores_mv[i]);
  }

  if (detectados.length() > 0) {
    Serial.print(" <-- ¡REFLEJO DETECTADO EN:" + detectados);
  }
  Serial.println();

  delay(150);
}
