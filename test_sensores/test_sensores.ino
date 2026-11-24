/*
  =============================================================================
  PROYECTO: SEGUIDOR DE LINEA PRO - TEST DE SENSORES CON ANTI-SATURACIÓN ADC
  =============================================================================
  SOLUCIÓN AL PROBLEMA DE "TODO EN 4095":
  - El ADC del ESP32-C3 satura (topa en 4095) a los 2922 mV (2.92V).
  - Tus barras de sensores tienen resistencias pull-up a 3.3V (~15 kOhm), lo que
    mantiene el voltaje en reposo cerca de 3.30V (encima del tope de 2.92V).
  - Por eso, aunque el sensor baje de 3.30V a 2.95V al detectar color/luz, el
    ADC sin carga sigue marcando 4095 fijo.
  - Al mantener activo el Pull-Down interno de 45 kOhm por registro de hardware
    (gpio_pulldown_en) durante la conversión ADC, formamos un divisor de tensión
    que baja el reposo a ~2480 mV (dentro del rango lineal del ADC) y permite
    ver cualquier cambio en cada sensor.
  =============================================================================
*/

#include "driver/gpio.h"

#define PIN_SIG      0   // GPIO0: Entrada analógica conectada a SIG (COM) del Mux
#define PIN_S0      20   // GPIO20: Selector S0
#define PIN_S1      10   // GPIO10: Selector S1
#define PIN_S2       7   // GPIO7: Selector S2
#define PIN_S3       6   // GPIO6: Selector S3
#define PIN_LED_ON   4   // GPIO4: Encendido de emisores IR

// Sensibilidad en milivoltios respecto al reposo para marcar detección [█]
const int SENSIBILIDAD_MV = 20;

uint16_t lecturas_mv[16];
uint16_t base_mv[16];
bool calibrado = false;

void seleccionarCanal(uint8_t canal) {
  digitalWrite(PIN_S0, (canal & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_S1, (canal & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_S2, (canal & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_S3, (canal & 0x08) ? HIGH : LOW);
  delayMicroseconds(30);
}

void leerSensoresMV() {
  // Asegurar que el Pull-Down de 45k esté activo en el pad para evitar saturación a 4095
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_en((gpio_num_t)PIN_SIG);

  for (uint8_t ch = 0; ch < 16; ch++) {
    seleccionarCanal(ch);
    delayMicroseconds(100);
    // Promediar 4 muestras rápidas para eliminar ruido
    uint32_t suma = 0;
    for (int m = 0; m < 4; m++) {
      suma += analogReadMilliVolts(PIN_SIG);
    }
    lecturas_mv[ch] = suma / 4;
  }
}

void calibrarBase() {
  for (int i = 0; i < 16; i++) base_mv[i] = 0;
  for (int muestra = 0; muestra < 10; muestra++) {
    leerSensoresMV();
    for (int i = 0; i < 16; i++) {
      base_mv[i] += lecturas_mv[i];
    }
    delay(20);
  }
  for (int i = 0; i < 16; i++) {
    base_mv[i] /= 10;
  }
  calibrado = true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);

  pinMode(PIN_LED_ON, OUTPUT);
  digitalWrite(PIN_LED_ON, HIGH);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Inicializar el bus ADC en PIN_SIG primero y luego activar el Pull-Down por hardware
  analogReadMilliVolts(PIN_SIG);
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_en((gpio_num_t)PIN_SIG);

  calibrarBase();
}

void loop() {
  leerSensoresMV();

  // Si el cable SIG se desconecta, con el Pull-Down caerá por debajo de 100 mV
  if (lecturas_mv[0] < 150 && lecturas_mv[7] < 150 && lecturas_mv[15] < 150) {
    Serial.println("[!] CABLE SIG DESCONECTADO O MUX SIN ALIMENTACION (Lectura: 0 mV)");
    delay(500);
    return;
  }

  Serial.println("------------------------------------------------------------------------------------------------------------------");
  Serial.println("CANAL:  | CH00  CH01  CH02  CH03  CH04  CH05  CH06  CH07 | CH08  CH09  CH10  CH11  CH12  CH13  CH14  CH15 |");
  Serial.println("SENSOR: | U3_S8 U3_S7 U3_S6 U3_S5 U3_S4 U3_S3 U3_S2 U3_S1 | U4_S8 U4_S7 U4_S6 U4_S5 U4_S4 U4_S3 U4_S2 U4_S1 |");
  Serial.println("------------------------------------------------------------------------------------------------------------------");

  // Mostrar voltaje real en mV (ya sin saturar en 4095)
  Serial.print("mV:     | ");
  for (int i = 0; i < 16; i++) {
    Serial.printf("%4d  ", lecturas_mv[i]);
    if (i == 7) Serial.print("| ");
  }
  Serial.println("|");

  // Mostrar diferencia (Delta) respecto al reposo
  Serial.print("CAMBIO: | ");
  for (int i = 0; i < 16; i++) {
    int diff = (int)lecturas_mv[i] - (int)base_mv[i];
    Serial.printf("%+4d  ", diff);
    if (i == 7) Serial.print("| ");
  }
  Serial.println("|");

  // Indicador visual cuando hay variación en el sensor
  Serial.print("DETECT: | ");
  for (int i = 0; i < 16; i++) {
    int diff = abs((int)lecturas_mv[i] - (int)base_mv[i]);
    if (diff >= SENSIBILIDAD_MV || lecturas_mv[i] < 2000) {
      Serial.print(" [█]  ");
    } else {
      Serial.print("  .   ");
    }
    if (i == 7) Serial.print("| ");
  }
  Serial.println("|");
  Serial.println("------------------------------------------------------------------------------------------------------------------\n");

  delay(200);
}
