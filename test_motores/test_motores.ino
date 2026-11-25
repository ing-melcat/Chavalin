/*
  =============================================================================
  TEST EXCLUSIVO DE MOTORES (TB6612FNG + ESP32-C3 SUPERMINI)
  =============================================================================
  Mapeo de pines (v1.kicad_sch):
    - AI1 (Motor Izquierdo / A) -> GPIO 1
    - AI2 (Motor Izquierdo / A) -> GPIO 2
    - BI1 (Motor Derecho / B)   -> GPIO 3
    - BI2 (Motor Derecho / B)   -> GPIO 8 (comparte con LED azul de la placa)
    - PWMA, PWMB, STBY          -> Fijos a 3.3V por hardware en PCB

  Método de control (PWM LEDC 20 kHz, 8 bits: 0..255):
    - Adelante: IN1 = PWM, IN2 = 0
    - Reversa:  IN1 = 0,   IN2 = PWM
    - Freno:    IN1 = 255, IN2 = 255
    - Paro:     IN1 = 0,   IN2 = 0

  Funcionamiento:
    - Inicia en modo MANUAL seguro (motores detenidos al arrancar).
    - Comandos por Monitor Serial (115200 baud):
        '1' -> Motor A (Izquierdo) ADELANTE (1.5 s)
        '2' -> Motor A (Izquierdo) REVERSA  (1.5 s)
        '3' -> Motor B (Derecho)   ADELANTE (1.5 s)
        '4' -> Motor B (Derecho)   REVERSA  (1.5 s)
        'w' -> AMBOS motores ADELANTE (1.5 s)
        'r' -> AMBOS motores REVERSA  (1.5 s)
        '+' / '-' -> Subir / bajar PWM de prueba (80..240)
        't' -> Iniciar ciclo automatico (5 fases)
        's' -> Paro inmediato / pausar ciclo
        'h' -> Mostrar menu de comandos
  =============================================================================
*/

#include <Arduino.h>

#define PIN_MOT_A1  1   // AI1: Motor Izquierdo (A)
#define PIN_MOT_A2  2   // AI2: Motor Izquierdo (A)
#define PIN_MOT_B1  3   // BI1: Motor Derecho (B)
#define PIN_MOT_B2  8   // BI2: Motor Derecho (B) - LED azul integrado

// Por seguridad mantenemos la turbina en 0% (1000 us) si el ESC está conectado en GPIO 5
#define PIN_TURBINA 5

const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 8;

bool ciclo_automatico = false;
int  pwm_test = 160;
int  curA = 0;
int  curB = 0;

void setMotorA(int speed) {
  speed = constrain(speed, -255, 255);
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
  curA = 0;
  curB = 0;
}

void irSuaveMotores(int tgtA, int tgtB) {
  while (curA != tgtA || curB != tgtB) {
    int nextA = tgtA;
    int nextB = tgtB;
    if ((curA > 0 && tgtA < 0) || (curA < 0 && tgtA > 0)) nextA = 0;
    if ((curB > 0 && tgtB < 0) || (curB < 0 && tgtB > 0)) nextB = 0;

    if (curA < nextA) curA = min(nextA, curA + 20);
    else if (curA > nextA) curA = max(nextA, curA - 20);

    if (curB < nextB) curB = min(nextB, curB + 20);
    else if (curB > nextB) curB = max(nextB, curB - 20);

    setMotors(curA, curB);
    delay(20);
  }
}

void brakeMotors() {
  ledcWrite(PIN_MOT_A1, 255);
  ledcWrite(PIN_MOT_A2, 255);
  ledcWrite(PIN_MOT_B1, 255);
  ledcWrite(PIN_MOT_B2, 255);
}

// Espera verificando si el usuario envía 's' para abortar al instante
bool esperarConAborto(uint32_t ms) {
  uint32_t inicio = millis();
  while (millis() - inicio < ms) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == 's' || c == 'S') {
        stopMotors();
        ciclo_automatico = false;
        Serial.println("\n>> [ALTO] Motores detenidos. Envia 't' para reanudar o '1'/'2'.");
        return false;
      }
    }
    delay(10);
  }
  return true;
}

void ejecutarSecuenciaMotores() {
  Serial.println("\n------------------------------------------------------");
  Serial.println(">> [1/5] MOTOR A (Izquierdo: GPIO 1/2) -> ADELANTE (PWM 140)");
  irSuaveMotores(140, 0);
  if (!esperarConAborto(1500)) return;
  irSuaveMotores(0, 0);
  if (!esperarConAborto(500)) return;

  Serial.println(">> [2/5] MOTOR A (Izquierdo: GPIO 1/2) -> REVERSA  (PWM -140)");
  irSuaveMotores(-140, 0);
  if (!esperarConAborto(1500)) return;
  irSuaveMotores(0, 0);
  if (!esperarConAborto(500)) return;

  Serial.println(">> [3/5] MOTOR B (Derecho:   GPIO 3/8) -> ADELANTE (PWM 140)");
  irSuaveMotores(0, 140);
  if (!esperarConAborto(1500)) return;
  irSuaveMotores(0, 0);
  if (!esperarConAborto(500)) return;

  Serial.println(">> [4/5] MOTOR B (Derecho:   GPIO 3/8) -> REVERSA  (PWM -140)");
  irSuaveMotores(0, -140);
  if (!esperarConAborto(1500)) return;
  irSuaveMotores(0, 0);
  if (!esperarConAborto(500)) return;

  Serial.println(">> [5/5] AMBOS MOTORES -> ADELANTE (PWM 160) + PARO SUAVE");
  irSuaveMotores(160, 160);
  if (!esperarConAborto(1500)) return;
  irSuaveMotores(0, 0);
  stopMotors();
  Serial.println(">> [OK] Ciclo terminado. Repitiendo en 3 s ('s' para detener)...");
}

void imprimirMenu() {
  Serial.println("\n======================================================");
  Serial.println("  TEST MANUAL DE MOTORES TB6612 - ESP32-C3 (COM5)");
  Serial.println("  Motor A (Izq): AI1=GPIO1, AI2=GPIO2");
  Serial.println("  Motor B (Der): BI1=GPIO3, BI2=GPIO8");
  Serial.printf ("  PWM actual: %d / 255 ('+' / '-' para ajustar)\n", pwm_test);
  Serial.println("  ------------------------------------------------------");
  Serial.println("  '1' = Motor IZQ ADELANTE | '2' = Motor IZQ REVERSA");
  Serial.println("  '3' = Motor DER ADELANTE | '4' = Motor DER REVERSA");
  Serial.println("  'w' = AMBOS ADELANTE     | 'r' = AMBOS REVERSA");
  Serial.println("  't' = Ciclo automatico   | 's' = PARO INMEDIATO");
  Serial.println("  'h' = Mostrar este menu");
  Serial.println("======================================================");
}

void probarManual(const char* desc, int sA, int sB, uint32_t ms) {
  ciclo_automatico = false;
  Serial.printf(">> [MANUAL PWM=%d] %s\n", pwm_test, desc);
  irSuaveMotores(sA, sB);
  if (esperarConAborto(ms)) {
    irSuaveMotores(0, 0);
    stopMotors();
    Serial.println(">> Detenido.");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(50);

  ledcAttach(PIN_MOT_A1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_A2, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_B1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_MOT_B2, PWM_FREQ, PWM_RES);
  stopMotors();

  // Mantener ESC de turbina en 1000 us (0%) por seguridad
  ledcAttach(PIN_TURBINA, 50, 14);
  ledcWrite(PIN_TURBINA, (1000UL * 16383UL) / 20000UL);

  delay(1500);
  imprimirMenu();
  Serial.println("Modo MANUAL listo (motores detenidos). Envia un comando:");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 's' || c == 'S') {
      stopMotors();
      ciclo_automatico = false;
      Serial.println(">> [ALTO] Motores detenidos (PWM=0).");
    } else if (c == 't' || c == 'T') {
      ciclo_automatico = true;
      Serial.println(">> [AUTO] Iniciando ciclo automatico de motores...");
    } else if (c == '1') {
      probarManual("Motor A (Izquierdo) ADELANTE (1.5 s)...", pwm_test, 0, 1500);
    } else if (c == '2') {
      probarManual("Motor A (Izquierdo) REVERSA  (1.5 s)...", -pwm_test, 0, 1500);
    } else if (c == '3') {
      probarManual("Motor B (Derecho) ADELANTE (1.5 s)...", 0, pwm_test, 1500);
    } else if (c == '4') {
      probarManual("Motor B (Derecho) REVERSA  (1.5 s)...", 0, -pwm_test, 1500);
    } else if (c == 'w' || c == 'W') {
      probarManual("AMBOS motores ADELANTE (1.5 s)...", pwm_test, pwm_test, 1500);
    } else if (c == 'r' || c == 'R') {
      probarManual("AMBOS motores REVERSA  (1.5 s)...", -pwm_test, -pwm_test, 1500);
    } else if (c == '+') {
      pwm_test = constrain(pwm_test + 20, 80, 240);
      Serial.printf(">> PWM de prueba = %d / 255\n", pwm_test);
    } else if (c == '-') {
      pwm_test = constrain(pwm_test - 20, 80, 240);
      Serial.printf(">> PWM de prueba = %d / 255\n", pwm_test);
    } else if (c == 'h' || c == 'H' || c == '?') {
      imprimirMenu();
    }
  }

  if (ciclo_automatico) {
    ejecutarSecuenciaMotores();
    esperarConAborto(3000);
  } else {
    delay(20);
  }
}
