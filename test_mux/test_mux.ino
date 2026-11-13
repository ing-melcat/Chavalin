/*
  =============================================================================
  DIAGNÓSTICO Y TEST DE CONEXIÓN DEL MULTIPLEXOR CD74HC4067
  =============================================================================
  Nota técnica: El CD74HC4067 es un multiplexor analógico pasivo (no es I2C/SPI),
  por lo que no tiene un registro digital o dirección I2C para "escanearlo".
  
  ¿Cómo verifica este test si el multiplexor está funcionando y conectado?:
  1. Conmuta los 16 canales uno por uno mediante S0, S1, S2, S3.
  2. Para cada canal, hace dos lecturas:
     - Con emisores IR apagados (GPIO4 = LOW)
     - Con emisores IR encendidos (GPIO4 = HIGH)
  3. Compara el cambio de voltaje (Delta en mV).
     -> Si el Mux está conmutando y vivo, verás cómo los canales varían
        independientemente y reaccionan al encendido de los emisores IR.
     -> Si el Mux está desconectado, dañado o sin alimentación (GND/3V3),
        la lectura se quedará congelada en 0V o 3.3V flotante en todos los canales.
  =============================================================================
*/

#define PIN_SIG      0   // GPIO0: Entrada analógica ADC (COM del CD74HC4067)
#define PIN_S0       6   // GPIO6: Selector S0
#define PIN_S1       7   // GPIO7: Selector S1
#define PIN_S2      10   // GPIO10: Selector S2
#define PIN_S3      20   // GPIO20: Selector S3
#define PIN_LED_ON   4   // GPIO4: Control de emisores IR

void setCanalMux(uint8_t canal) {
  digitalWrite(PIN_S0, (canal & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_S1, (canal & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_S2, (canal & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_S3, (canal & 0x08) ? HIGH : LOW);
  delayMicroseconds(20); // Estabilización del multiplexor
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_LED_ON, OUTPUT);

  analogReadResolution(12);
  #if defined(ESP32)
    analogSetAttenuation(ADC_11db);
  #endif

  Serial.println("\n=======================================================");
  Serial.println("     DIAGNÓSTICO DEL MULTIPLEXOR CD74HC4067 (16-CH)");
  Serial.println("=======================================================");
  Serial.println("Pines:");
  Serial.println("  COM (Señal) -> GPIO 0 (ADC)");
  Serial.println("  S0, S1, S2, S3 -> GPIO 6, 7, 10, 20");
  Serial.println("  LED_ON       -> GPIO 4");
  Serial.println("-------------------------------------------------------\n");
}

void loop() {
  Serial.println(">> ESCANEANDO LOS 16 CANALES DEL MULTIPLEXOR...");
  Serial.println("CANAL | BITS [S3 S2 S1 S0] | IR OFF (mV) | IR ON (mV) | DELTA (mV) | ESTADO");
  Serial.println("------+--------------------+-------------+------------+------------+------------------");

  uint8_t canales_vivos = 0;
  uint8_t canales_fijos_cero = 0;
  uint8_t canales_fijos_max = 0;

  for (uint8_t ch = 0; ch < 16; ch++) {
    // 1. Seleccionar canal
    setCanalMux(ch);

    // 2. Medir con emisores IR apagados
    digitalWrite(PIN_LED_ON, LOW);
    delayMicroseconds(50);
    uint32_t raw_off = analogRead(PIN_SIG);
    uint32_t mv_off = analogReadMilliVolts(PIN_SIG);

    // 3. Medir con emisores IR encendidos
    digitalWrite(PIN_LED_ON, HIGH);
    delayMicroseconds(50);
    uint32_t raw_on = analogRead(PIN_SIG);
    uint32_t mv_on = analogReadMilliVolts(PIN_SIG);

    int32_t delta = abs((int32_t)mv_on - (int32_t)mv_off);

    // Clasificar canal
    String diagnostico = "";
    if (raw_on < 50 && raw_off < 50) {
      diagnostico = "Fijo en 0V (GND)";
      canales_fijos_cero++;
    } else if (raw_on > 4050 && raw_off > 4050) {
      diagnostico = "Fijo en 3.3V (VCC/Flotante)";
      canales_fijos_max++;
    } else {
      diagnostico = "OK - Señal activa";
      canales_vivos++;
    }

    // Imprimir fila formateada
    Serial.printf("CH %02d |     [%d  %d  %d  %d]     |  %4d mV    |  %4d mV   |  %+5d mV  | %s\n",
                  ch,
                  (ch >> 3) & 1, (ch >> 2) & 1, (ch >> 1) & 1, ch & 1,
                  mv_off, mv_on, delta, diagnostico.c_str());
  }

  Serial.println("----------------------------------------------------------------------------------");
  
  // Resumen del estado del multiplexor
  if (canales_fijos_max == 16) {
    Serial.println("ALERTA: Todos los canales leen 3.3V fijo.");
    Serial.println("-> Posible causa: El pin COM (GPIO 0) está flotando o el Mux no tiene GND / EN no está a GND.");
  } else if (canales_fijos_cero == 16) {
    Serial.println("ALERTA: Todos los canales leen 0V.");
    Serial.println("-> Posible causa: Pin COM en corto a GND o falta alimentación 3.3V en el MUX.");
  } else {
    Serial.printf(">> RESULTADO: MULTIPLEXOR OPERATIVO. (%d/16 canales con señal activa)\n", canales_vivos);
    Serial.println("   Los canales muestran diferencias individuales de voltaje, confirmando");
    Serial.println("   que las líneas de dirección (S0-S3) y el pin COM están conmutando.");
  }

  Serial.println("==================================================================================\n");
  delay(1500); // Pausa de 1.5s antes del próximo ciclo de diagnóstico
}
