/*
  =============================================================================
  PROYECTO: SEGUIDOR DE LINEA PRO - INTEGRACIÓN DE SENSORES Y DRIVER TB6612
  =============================================================================
  Hardware:
    Microcontrolador: ESP32-C3 SuperMini (COM5)
    Driver de Motores: TB6612 (U1)
      - AI1 -> GPIO 1 (Motor Izquierdo / A)
      - AI2 -> GPIO 2 (Motor Izquierdo / A)
      - BI1 -> GPIO 3 (Motor Derecho / B)
      - BI2 -> GPIO 8 (Motor Derecho / B)
      - PWMA, PWMB, STBY -> Conectados permanentemente a 3.3V (VCC) en esquemático.
      - VM -> Batería 12V (BT1)
    Multiplexor: CD74HC4067 (U5)
      - SIG (COM) -> GPIO 0 (ADC con hardware pull-down anti-saturación)
      - S0, S1, S2, S3 -> GPIO 6, 7, 10, 20
      - LED_ON -> GPIO 4
      - EN -> GND
    Sensores: 2x Barras QTR-8A (16 canales analógicos)
  =============================================================================
*/

#include <Arduino.h>
#include "driver/gpio.h"

// --- PINES DEL DRIVER TB6612 ---
#define PIN_MOT_A1   1   // GPIO 1: Control Motor A (AI1)
#define PIN_MOT_A2   2   // GPIO 2: Control Motor A (AI2)
#define PIN_MOT_B1   3   // GPIO 3: Control Motor B (BI1)
#define PIN_MOT_B2   8   // GPIO 8: Control Motor B (BI2) - Nota: LED azul de la placa comparte GPIO8

// Frecuencia PWM silenciosa (20 kHz) y resolución de 8 bits (0 - 255)
const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 8;

// --- PINES DEL MULTIPLEXOR Y SENSORES ---
#define PIN_SIG      0   // GPIO 0: Entrada ADC (COM/SIG del CD74HC4067)
#define PIN_S0       6   // GPIO 6: Selector S0
#define PIN_S1       7   // GPIO 7: Selector S1
#define PIN_S2      10   // GPIO 10: Selector S2
#define PIN_S3      20   // GPIO 20: Selector S3
#define PIN_LED_ON   4   // GPIO 4: Control de emisores IR de las barras

// --- VARIABLES DE SENSORES ---
uint16_t lecturas_mv[16];
uint16_t base_mv[16];
const int UMBRAL_DELTA = 25; // Milivoltios de cambio para considerar que detectó línea

// Estado de control
bool modo_seguidor_activo = false;

// -----------------------------------------------------------------------------
// CONTROL DE MOTORES TB6612
// Al estar PWMA y PWMB a 3.3V, el control de velocidad y dirección se hace por PWM
// directo sobre las entradas IN1 e IN2 (Forward: IN1=PWM, IN2=0; Reverse: IN1=0, IN2=PWM)
// -----------------------------------------------------------------------------

void setMotorA(int speed) {
  // speed: -255 a +255 (positivo = adelante, negativo = reversa, 0 = paro)
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    ledcWrite(PIN_MOT_A1, speed);
    ledcWrite(PIN_MOT_A2, 0);
  } else if (speed < 0) {
    ledcWrite(PIN_MOT_A1, 0);
    ledcWrite(PIN_MOT_A2, -speed);
  } else {
    // Paro (coasting)
    ledcWrite(PIN_MOT_A1, 0);
    ledcWrite(PIN_MOT_A2, 0);
  }
}

void setMotorB(int speed) {
  // speed: -255 a +255
  speed = constrain(speed, -255, 255);
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
  // Freno dinámico corto (ambas entradas en HIGH)
  ledcWrite(PIN_MOT_A1, 255);
  ledcWrite(PIN_MOT_A2, 255);
  ledcWrite(PIN_MOT_B1, 255);
  ledcWrite(PIN_MOT_B2, 255);
}

// -----------------------------------------------------------------------------
// RUTINA DE PRUEBA DE MOTORES
// -----------------------------------------------------------------------------
void ejecutarTestMotores() {
  Serial.println("\n>> [TEST MOTORES] Iniciando secuencia de prueba (3 segundos de preparación)...");
  stopMotors();
  delay(1000);
  Serial.println(">> 2...");
  delay(1000);
  Serial.println(">> 1...");
  delay(1000);

  // 1. Motor A adelante suave
  Serial.println(">> 1/5: Motor A (Izquierdo) ADELANTE (50% velocidad)...");
  setMotorA(130);
  setMotorB(0);
  delay(1200);
  stopMotors();
  delay(400);

  // 2. Motor B adelante suave
  Serial.println(">> 2/5: Motor B (Derecho) ADELANTE (50% velocidad)...");
  setMotorA(0);
  setMotorB(130);
  delay(1200);
  stopMotors();
  delay(400);

  // 3. Ambos motores adelante
  Serial.println(">> 3/5: AMBOS MOTORES ADELANTE (40% velocidad)...");
  setMotors(100, 100);
  delay(1200);
  stopMotors();
  delay(400);

  // 4. Ambos motores reversa
  Serial.println(">> 4/5: AMBOS MOTORES REVERSA (40% velocidad)...");
  setMotors(-100, -100);
  delay(1200);
  stopMotors();
  delay(400);

  // 5. Giro en su propio eje
  Serial.println(">> 5/5: GIRO EN SU PROPIO EJE (Izquierda)...");
  setMotors(-100, 100);
  delay(800);
  stopMotors();

  Serial.println(">> [TEST MOTORES] ¡Secuencia completada con éxito! Motores en reposo.\n");
}

// -----------------------------------------------------------------------------
// LECTURA DE SENSORES Y MULTIPLEXOR
// -----------------------------------------------------------------------------
void seleccionarCanalMux(uint8_t canal) {
  digitalWrite(PIN_S0, (canal & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_S1, (canal & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_S2, (canal & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_S3, (canal & 0x08) ? HIGH : LOW);
  delayMicroseconds(25);
}

void leerSensores() {
  // Asegurar que el Pull-Down por hardware esté activo en el pad
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_en((gpio_num_t)PIN_SIG);

  for (uint8_t ch = 0; ch < 16; ch++) {
    seleccionarCanalMux(ch);
    delayMicroseconds(60);
    // 2 muestras para filtrado rápido
    uint32_t v1 = analogReadMilliVolts(PIN_SIG);
    uint32_t v2 = analogReadMilliVolts(PIN_SIG);
    lecturas_mv[ch] = (v1 + v2) / 2;
  }
}

void calibrarBase() {
  Serial.println(">> Calibrando valores base de los 16 sensores...");
  for (int i = 0; i < 16; i++) base_mv[i] = 0;
  for (int m = 0; m < 15; m++) {
    leerSensores();
    for (int i = 0; i < 16; i++) {
      base_mv[i] += lecturas_mv[i];
    }
    delay(15);
  }
  for (int i = 0; i < 16; i++) {
    base_mv[i] /= 15;
  }
  Serial.println(">> Calibracion completada.\n");
}

// Calcula el centroide / error de la línea (-700 a +700)
// Negativo: Línea a la izquierda | Positivo: Línea a la derecha
float calcularErrorLinea() {
  long suma_ponderada = 0;
  long suma_pesos = 0;

  for (int i = 0; i < 16; i++) {
    int delta = abs((int)lecturas_mv[i] - (int)base_mv[i]);
    if (delta > UMBRAL_DELTA) {
      // Posición física de sensores de izquierda a derecha (-7.5 a +7.5 multiplicado por 100)
      // Ch 0..7 es Barra 1 (descendente U3_LED8..U3_LED1), Ch 8..15 es Barra 2 (U4_LED8..U4_LED1)
      int posicion = (i - 7) * 100 - 50; 
      suma_ponderada += (long)posicion * delta;
      suma_pesos += delta;
    }
  }

  if (suma_pesos == 0) return 0; // Sin línea detectada
  return (float)suma_ponderada / suma_pesos;
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1200);

  // Inicializar pines del Driver TB6612 con PWM LEDC
  ledcAttach(PIN_MOT_A1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_A2, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_B1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_B2, PWM_FREQ, PWM_RES);
  stopMotors(); // Garantizar motores detenidos al arrancar

  // Inicializar pines de multiplexor y sensores
  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_LED_ON, OUTPUT);
  digitalWrite(PIN_LED_ON, HIGH); // Encender emisores IR

  // Inicializar ADC y aplicar Anti-Saturación de hardware
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogReadMilliVolts(PIN_SIG);
  gpio_pullup_dis((gpio_num_t)PIN_SIG);
  gpio_pulldown_en((gpio_num_t)PIN_SIG);

  calibrarBase();

  Serial.println("==================================================================");
  Serial.println("  SEGUIDOR DE LINEA PRO v1 - SENSORES + DRIVER TB6612 INTEGRADOS");
  Serial.println("==================================================================");
  Serial.println("Comandos disponibles en el Monitor Serie (enviar caracter):");
  Serial.println("  't' o 'm' -> Ejecutar rutina de prueba de motores");
  Serial.println("  's'       -> PARO DE EMERGENCIA (Detener motores)");
  Serial.println("  '1'       -> Probar solo Motor A (Adelante 1 segundo)");
  Serial.println("  '2'       -> Probar solo Motor B (Adelante 1 segundo)");
  Serial.println("  'c'       -> Recalibrar sensores en reposo");
  Serial.println("  'l'       -> Alternar modo Seguidor de Línea ON / OFF");
  Serial.println("==================================================================\n");
}

// -----------------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------------
void loop() {
  // 1. Procesar comandos del Monitor Serie
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 't' || cmd == 'm') {
      ejecutarTestMotores();
    } else if (cmd == 's') {
      stopMotors();
      modo_seguidor_activo = false;
      Serial.println(">> [ALTO] Motores detenidos.");
    } else if (cmd == '1') {
      Serial.println(">> Probando Motor A adelante...");
      setMotorA(120);
      delay(1000);
      setMotorA(0);
      Serial.println(">> Motor A detenido.");
    } else if (cmd == '2') {
      Serial.println(">> Probando Motor B adelante...");
      setMotorB(120);
      delay(1000);
      setMotorB(0);
      Serial.println(">> Motor B detenido.");
    } else if (cmd == 'c') {
      calibrarBase();
    } else if (cmd == 'l') {
      modo_seguidor_activo = !modo_seguidor_activo;
      if (!modo_seguidor_activo) stopMotors();
      Serial.printf(">> Modo Seguidor de Linea: %s\n", modo_seguidor_activo ? "ACTIVADO" : "DESACTIVADO");
    }
  }

  // 2. Leer sensores
  leerSensores();
  float error_linea = calcularErrorLinea();

  // 3. Control seguidor de línea proporcional básico (si está activado con 'l')
  if (modo_seguidor_activo) {
    int velocidad_base = 120;
    float kp = 0.35;
    int correccion = (int)(error_linea * kp);

    int vel_A = velocidad_base + correccion;
    int vel_B = velocidad_base - correccion;
    setMotors(vel_A, vel_B);
  }

  // 4. Visualización en Monitor Serie (cada ~200ms)
  static uint32_t ultimo_print = 0;
  if (millis() - ultimo_print > 200) {
    ultimo_print = millis();

    Serial.print("SENSORES: [");
    int sensores_activos = 0;
    for (int i = 0; i < 16; i++) {
      int delta = abs((int)lecturas_mv[i] - (int)base_mv[i]);
      if (delta >= UMBRAL_DELTA || lecturas_mv[i] < 2000) {
        Serial.print("█");
        sensores_activos++;
      } else {
        Serial.print(".");
      }
      if (i == 7) Serial.print(" | ");
    }
    Serial.printf("] | Error: %+4.0f | MotA: %3d | MotB: %3d | Modo: %s\n",
                  error_linea,
                  modo_seguidor_activo ? 120 : 0,
                  modo_seguidor_activo ? 120 : 0,
                  modo_seguidor_activo ? "AUTO" : "MANUAL");
  }

  delay(10);
}
