#include <Servo.h>

// ============================================================
// Prueba de banco - Turbina EDF (QX-Motor 14000KV + 2S 450mAh)
// Placa: Arduino Nano (ATmega328P)
// ============================================================

#define PIN_ESC 9        // Pin de señal del ESC (D9)
#define US_MIN  1000     // 0% (Armado / Paro)
#define US_MAX  2000     // 100%
#define MAX_PCT 25       // Límite de seguridad para esta prueba (%)

Servo esc;
int currentUs = US_MIN;

void setThrottleInstant(int us) {
  currentUs = constrain(us, US_MIN, US_MAX);
  esc.writeMicroseconds(currentUs);
}

void rampToPct(int pct) {
  pct = constrain(pct, 0, MAX_PCT);
  int targetUs = map(pct, 0, 100, US_MIN, US_MAX);

  if (targetUs == currentUs) return;

  // Si baja a 0%, corte inmediato por seguridad
  if (pct == 0) {
    setThrottleInstant(US_MIN);
    Serial.println(F("[PARO] 0% (1000 us)"));
    return;
  }

  // Rampa suave hacia arriba/abajo para no dar tirones a la batería 2S
  int step = (targetUs > currentUs) ? 5 : -5;
  while (currentUs != targetUs) {
    currentUs += step;
    if ((step > 0 && currentUs > targetUs) || (step < 0 && currentUs < targetUs)) {
      currentUs = targetUs;
    }
    esc.writeMicroseconds(currentUs);
    delay(15);
  }

  Serial.print(F("[OK] Potencia: "));
  Serial.print(pct);
  Serial.print(F("% ("));
  Serial.print(currentUs);
  Serial.println(F(" us)"));
}

void printMenu() {
  Serial.println(F("\n--- Comandos (Monitor Serial 115200, Sin ajuste / NL) ---"));
  Serial.println(F("  0  (o 's') -> PARO INMEDIATO (0%)"));
  Serial.println(F("  10         -> Probar al 10% (1100 us)"));
  Serial.println(F("  25         -> Probar al 25% (1250 us)"));
  Serial.println(F("  1..25      -> Cualquier porcentaje hasta 25%"));
  Serial.println(F("---------------------------------------------------------"));
}

void setup() {
  Serial.begin(115200);
  esc.attach(PIN_ESC, US_MIN, US_MAX);
  setThrottleInstant(US_MIN);

  Serial.println(F("\n=== Prueba Turbina EDF - Arduino Nano ==="));
  Serial.println(F("Enviando 1000 us (0%) para armar el ESC..."));
  Serial.println(F("Conecta la bateria 2S al ESC ahora si aun no lo hiciste."));
  delay(3500);

  Serial.println(F("ESC listo (si escuchaste los pitidos de armado)."));
  printMenu();
}

void loop() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  // Paro de emergencia con 's', 'S', o '0'
  if (input.equalsIgnoreCase("s") || input == "0") {
    rampToPct(0);
    return;
  }

  // Validar que sea numérico
  for (unsigned int i = 0; i < input.length(); i++) {
    if (!isDigit(input[i])) {
      Serial.println(F("[AVISO] Comando no reconocido -> PARO por seguridad."));
      rampToPct(0);
      return;
    }
  }

  int pct = input.toInt();
  if (pct > MAX_PCT) {
    Serial.print(F("[LIMITE] Tope de seguridad en "));
    Serial.print(MAX_PCT);
    Serial.println(F("%. Aplicando 25%."));
    pct = MAX_PCT;
  }

  rampToPct(pct);
}
