/*
 * =====================================================================
 * ESP32 PID-Füllstandsregelung für Tank mit Befüllungsventil & Ablauf
 * Generiert für Arduino IDE / PlatformIO
 * =====================================================================
 * Konfiguration:
 * - Sensor: JSN-SR04T / HC-SR04 Ultraschall
 * - Stellglied: Proportionalventil / Pumpe via PWM (LEDC)
 * - Regler: PID mit Anti-Windup und Derivative-on-Measurement
 * - Timing: FreeRTOS Task mit vTaskDelayUntil
 * =====================================================================
 */

#include <Arduino.h>

// ======================= PIN-BELEGUNG =======================
#define PIN_TRIGGER       5   // Trigger-Signal (Ausgang, 10µs Puls)
#define PIN_ECHO          18   // Echo-Signal (Eingang - Pegelwandler 5V->3.3V beachten!)

#define PIN_ACTUATOR      25   // Ventil-Ausgang (MOSFET / Treiber-Stufe)

// ================== PHYSIKALISCHE TANKDATEN ==================
const float TANK_HEIGHT_CM    = 100.0f;  // Gesamthöhe des Tanks in cm
const float SENSOR_OFFSET_CM  = 5.0f;   // Distanz Sensor zu maximalem Füllstand (Totzone)

// ======================= PID-PARAMETER =======================
float Kp = 3.500f;       // Proportionalbeiwert
float Ki = 0.1500f;       // Integralbeiwert (1/s)
float Kd = 0.8000f;       // Differenzialbeiwert (s)

float setpoint = 60.0f;  // Soll-Füllstand in cm
const float DEADBAND_CM = 0.30f; // Totzone gegen Ventilflattern
const unsigned long SAMPLE_TIME_MS = 100; // Abtastzeit in Millisekunden

// ===================== REGLER-VARIABLEN =====================
float currentLevel = 0.0f;      // Istwert (PV)
float controlOutput = 0.0f;     // Stellgröße 0.0 - 100.0%
float integralSum = 0.0f;       // Integrierter Fehler
float lastLevel = 0.0f;         // Für Ableitung
float lastError = 0.0f;         // Für D-Anteil bei Fehlersprung
float dFiltered = 0.0f;         // Gefilterte Ableitung


// ==================== FUNKTIONS-PROTOTYPEN ====================
float readSensorLevel();
float computePID(float sp, float pv, float dt);
void writeActuator(float percent);


// =========================== SETUP ===========================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F(">>> ESP32 Tank-PID-Regelung initialisiert <<<"));

  // Pins konfigurieren
  pinMode(PIN_TRIGGER, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIGGER, LOW);

  // Aktor initialisieren
  // ESP32 LEDC PWM Konfiguration (Core v2/v3 kompatibel)
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(PIN_ACTUATOR, 5000, 10);
  #else
    ledcSetup(0, 5000, 10); // Kanal 0, 5000Hz, 10 Bit
    ledcAttachPin(PIN_ACTUATOR, 0);
  #endif



  // FreeRTOS PID-Regler Task auf Core 1 starten
  xTaskCreatePinnedToCore(
    pidControlTask,     // Task-Funktion
    "PID_Task",         // Name
    4096,               // Stackgröße
    NULL,               // Parameter
    2,                  // Priorität
    NULL,               // Task Handle
    1                   // Core 1 (Core 0 für WiFi)
  );

}

// ======================= REGLER-SCHLEIFE =====================
void pidControlTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_TIME_MS);
  float dt = (float)SAMPLE_TIME_MS / 1000.0f;

  for (;;) {
    // 1. Füllstand einlesen (gefilterter Istwert)
    currentLevel = readSensorLevel();

    // 2. PID-Regelalgorithmus berechnen
    controlOutput = computePID(setpoint, currentLevel, dt);

    // 3. Stellglied ansteuern (0 - 100%)
    writeActuator(controlOutput);

    // 4. Telemetrie ausgeben (Arduino Serial Plotter kompatibel)
    Serial.printf("Sollwert:%.2f,Istwert:%.2f,Ventil_PWM:%.2f\n", setpoint, currentLevel, controlOutput);

    // Exakter Zeittakt
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void loop() {
  // FreeRTOS übernimmt den Regler-Task
  vTaskDelay(pdMS_TO_TICKS(50));
}

// ================== SENSOR-MESSUNG & FILTER ==================
float readSensorLevel() {
  // 5-Punkte gleitender Durchschnitt gegen Ultraschall-Ausreißer
  float sum = 0.0f;
  const int SAMPLES = 3;
  int validSamples = 0;

  for (int i = 0; i < SAMPLES; i++) {
    digitalWrite(PIN_TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIGGER, LOW);

    // Echo ausmessen (Timeout 30ms ca. 5 Meter Reichweite)
    long duration = pulseIn(PIN_ECHO, HIGH, 30000);
    if (duration > 0) {
      // Schallgeschwindigkeit in Luft: ca. 0.0343 cm/µs
      float measuredDist = (duration * 0.0343f) / 2.0f;
      // Füllstand = Gesamthöhe - gemessener Freiraum
      float level = TANK_HEIGHT_CM - (measuredDist - SENSOR_OFFSET_CM);
      level = constrain(level, 0.0f, TANK_HEIGHT_CM);
      sum += level;
      validSamples++;
    }
    delay(10);
  }

  if (validSamples > 0) {
    return sum / validSamples;
  }
  return currentLevel; // Bei Messausfall vorherigen Wert beibehalten
}

// =================== PID-REGELALGORITHMUS ====================
float computePID(float sp, float pv, float dt) {
  float error = sp - pv;

  // 1. Totbandprüfung (verhindert ständiges Nachregeln bei Ruhelage)
  if (fabs(error) < DEADBAND_CM) {
    error = 0.0f;
  }

  // 2. Proportional-Anteil (P)
  float pTerm = Kp * error;

  // 3. Vorläufiger Integral-Anteil (I)
  float deltaI = Ki * error * dt;
  float tentativeI = integralSum + deltaI;

  // 4. Differenzial-Anteil (D) mit Tiefpassfilter (N=10)
  float rawDerivative = 0.0f;
  if (dt > 0.0001f) {
    // Derivative-on-Measurement: verhindert Reglersprung bei Sollwertänderung
    rawDerivative = -(pv - lastLevel) / dt;
  }

  // Tiefpassfilterung des D-Anteils gegen hochfrequentes Sensorrauschen
  const float N = 10.0f;
  float alpha = (N * dt) / (1.0f + N * dt);
  dFiltered = (1.0f - alpha) * dFiltered + alpha * rawDerivative;
  float dTerm = Kd * dFiltered;

  // 5. Gesamt-Stellgröße vor Begrenzung
  float rawOutput = pTerm + tentativeI + dTerm;

  // 6. Anti-Windup Clamping (Verhindert Überlaufen des Integrators bei Sättigung)
  bool saturatingHigh = (rawOutput > 100.0f && error > 0.0f);
  bool saturatingLow  = (rawOutput < 0.0f   && error < 0.0f);

  if (!saturatingHigh && !saturatingLow) {
    integralSum = tentativeI;
  }

  // Werte für nächsten Zeitschritt merken
  lastLevel = pv;
  lastError = error;

  // 7. Stellgröße auf 0.0 - 100.0% beschränken
  return constrain(pTerm + integralSum + dTerm, 0.0f, 100.0f);
}

// =================== STELLGLIED-ANSTEUERUNG ==================
void writeActuator(float percent) {
  // 0 - 100% auf LEDC PWM (0 - 1023) abbilden
  const uint32_t maxDuty = (1 << 10) - 1;
  uint32_t duty = (uint32_t)((percent / 100.0f) * maxDuty);

  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(PIN_ACTUATOR, duty);
  #else
    ledcWrite(0, duty); // LEDC Kanal 0
  #endif
}

