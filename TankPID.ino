/*
  ============================================================================
  TankPID – Füllstandsregelung eines Tanks mit PID-Regler auf dem ESP32
  ============================================================================
  RwTec · Arduino IDE · ESP32 (Arduino-Core 2.x und 3.x)

  Aufgabe:
    Der Füllstand im Tank soll auf dem Sollwert gehalten werden, auch wenn
    sich der Ablauf ändert (Störgröße). Der Regler verstellt dazu das
    Zulaufventil (Stellgröße 0–100 % → 0–10 V).

  Regelkreis:
    Sollwert ──►(Σ)──► PID ──► Zulaufventil ──► Tank ──┬──► Istwert
                ▲ −                                    │
                └────────── Füllstandssensor ◄─────────┘

  Hardware (Standard, siehe README.md):
    - Füllstandssensor 4–20 mA über Shunt 120 Ω an GPIO 34 (ADC1)
      oder Ultraschallsensor JSN-SR04T an GPIO 26 (Trig) / 27 (Echo)
    - Proportionalventil 0–10 V: PWM an GPIO 25 → RC-Filter → OPV (×3)
    - Störungs-LED an GPIO 2

  Bedienung über den seriellen Monitor (115200 Baud), Befehl "?" zeigt Hilfe.
  Für den seriellen Plotter: Befehl "plot" schaltet die Plot-Ausgabe ein.
  ============================================================================
*/

#include <Arduino.h>
#include <Preferences.h>

// ============================================================================
// 1) KONFIGURATION – hier an die eigene Anlage anpassen
// ============================================================================

// ---- Sensortyp wählen -------------------------------------------------------
#define SENSOR_4_20MA      1
#define SENSOR_ULTRASCHALL 2
#define SENSOR_TYP         SENSOR_4_20MA

// ---- Pins ------------------------------------------------------------------
const int PIN_VENTIL_PWM   = 25;   // PWM-Ausgang zum RC-Filter / OPV
const int PIN_SENSOR_ADC   = 34;   // 4–20 mA über Shunt (nur ADC1-Pins nutzen!)
const int PIN_US_TRIG      = 26;   // Ultraschall Trigger
const int PIN_US_ECHO      = 27;   // Ultraschall Echo (5 V → Spannungsteiler!)
const int PIN_LED_STOERUNG = 2;    // Onboard-LED: leuchtet bei Störung

// ---- Tank -------------------------------------------------------------------
const float TANK_HOEHE_CM      = 100.0f;  // nutzbare Höhe
const float NOT_HOCH_CM        = 95.0f;   // ab hier Zulauf sofort ZU (Überlaufschutz)
const float NOT_HOCH_RESET_CM  = 92.0f;   // Hysterese: darunter wieder freigeben

// ---- 4–20 mA-Sensor ---------------------------------------------------------
const float SHUNT_OHM          = 120.0f;  // 20 mA → 2,40 V (ADC-Reserve bis ~3,1 V)
const float SENSOR_0_CM        = 0.0f;    // Füllstand bei 4 mA
const float SENSOR_20MA_CM     = 100.0f;  // Füllstand bei 20 mA
const float MA_FEHLER_UNTEN    = 3.6f;    // NAMUR NE43: < 3,6 mA = Leitungsbruch
const float MA_FEHLER_OBEN     = 21.0f;   // NAMUR NE43: > 21 mA = Kurzschluss

// ---- Ultraschallsensor (Montage oben, misst nach unten) ---------------------
const float US_ABSTAND_LEER_CM = 125.0f;  // Abstand Sensor → Tankboden
const float US_MIN_CM          = 22.0f;   // Totzone JSN-SR04T (~20 cm)
const uint32_t US_TIMEOUT_US   = 12000;   // ~2 m Laufweg, blockiert max. 12 ms

// ---- Ventilausgang ----------------------------------------------------------
const uint32_t PWM_FREQ_HZ     = 19500;   // 12 Bit bei 80 MHz → max. ~19,5 kHz
const uint8_t  PWM_BITS        = 12;
const float VENTIL_MIN_PROZENT = 0.0f;    // Stellbereich begrenzen
const float VENTIL_MAX_PROZENT = 100.0f;
const float VENTIL_KAL_FAKTOR  = 1.000f;  // Feinabgleich, damit 100 % = 10,00 V

// ---- Regler-Standardwerte (Einheiten wie im Simulator) ----------------------
const float STD_SOLLWERT_CM = 83.0f;
const float STD_KP          = 10.0f;      // % pro cm
const float STD_KI          = 0.10f;      // % pro (cm · s)
const float STD_KD          = 0.0f;       // % · s pro cm
const float D_FILTER_TAU_S  = 2.0f;       // Tiefpass für D-Anteil
const float MESS_FILTER_S   = 0.5f;       // Tiefpass für den Messwert

const uint32_t ABTASTZEIT_MS = 100;       // Reglertakt 10 Hz

// ============================================================================
// 2) PID-REGLER
// ============================================================================
//  u = Kp·e + Ki·∫e dt − Kd·d(Ist)/dt
//
//  - D-Anteil wirkt auf den Istwert statt auf e: Ein Sollwertsprung erzeugt
//    so keinen "Derivative Kick" (keinen Ventilschlag).
//  - D-Anteil wird gefiltert, sonst zappelt das Ventil bei Messrauschen.
//  - Anti-Windup: Ist der Ausgang am Anschlag, wird nicht weiter integriert.
//  - reset() sorgt für stoßfreie Umschaltung Hand → Automatik.
// ----------------------------------------------------------------------------
class PIDRegler {
public:
  float kp = 0, ki = 0, kd = 0;
  float ausMin = 0, ausMax = 100;
  float dTau = 1.0f;

  // Anteile zum Anzeigen
  float anteilP = 0, anteilI = 0, anteilD = 0;

  void reset(float istwert, float ausgang) {
    integral = constrain(ausgang, ausMin, ausMax);
    istAlt = istwert;
    dFilt = 0;
    erstAufruf = true;
  }

  float berechne(float soll, float ist, float dt) {
    if (dt <= 0) dt = 0.001f;
    const float e = soll - ist;

    // P – "Wie weit bin ich weg?"
    const float p = kp * e;

    // D – "Wie schnell ändert sich der Istwert?" (gefiltert)
    float dRoh = erstAufruf ? 0.0f : -(ist - istAlt) / dt;
    erstAufruf = false;
    istAlt = ist;
    const float a = dt / (dTau + dt);
    dFilt += a * (dRoh - dFilt);
    const float d = kd * dFilt;

    // I – "Wie lange bin ich schon daneben?" mit Anti-Windup
    const float iNeu = integral + ki * e * dt;
    float u = p + iNeu + d;
    const bool obenSatt  = (u > ausMax) && (e > 0);
    const bool untenSatt = (u < ausMin) && (e < 0);
    if (!obenSatt && !untenSatt) {
      integral = iNeu;            // nur integrieren, wenn es noch etwas bringt
    }
    integral = constrain(integral, ausMin, ausMax);
    u = p + integral + d;

    anteilP = p; anteilI = integral; anteilD = d;
    return constrain(u, ausMin, ausMax);
  }

private:
  float integral = 0, istAlt = 0, dFilt = 0;
  bool erstAufruf = true;
};

// ============================================================================
// 3) GLOBALE ZUSTÄNDE
// ============================================================================
enum class Betrieb : uint8_t { AUTO, HAND };

PIDRegler   pid;
Preferences nvs;

float   sollwert     = STD_SOLLWERT_CM;
float   istwert      = 0;        // gefilterter Füllstand in cm
float   ventil       = 0;        // aktuelle Stellgröße in %
float   handWert     = 0;        // Ventilwert im Handbetrieb
Betrieb betrieb      = Betrieb::AUTO;
bool    sensorFehler = false;
bool    notHoch      = false;
bool    plotAusgabe  = false;
bool    istInit      = false;

uint32_t letzterTakt = 0;
uint32_t letzteAusgabe = 0;
uint32_t letzterTaktUs = 0;

// ============================================================================
// 4) VENTILAUSGANG (PWM → RC-Filter → OPV → 0–10 V)
// ============================================================================
void ventilInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_VENTIL_PWM, PWM_FREQ_HZ, PWM_BITS);
#else
  ledcSetup(0, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(PIN_VENTIL_PWM, 0);
#endif
}

void ventilSetzen(float prozent) {
  prozent = constrain(prozent, 0.0f, 100.0f);
  const uint32_t maxDuty = (1UL << PWM_BITS) - 1;
  uint32_t duty = (uint32_t)lroundf(prozent / 100.0f * VENTIL_KAL_FAKTOR * maxDuty);
  if (duty > maxDuty) duty = maxDuty;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_VENTIL_PWM, duty);
#else
  ledcWrite(0, duty);
#endif
}

// ============================================================================
// 5) FÜLLSTAND MESSEN
// ============================================================================
// Rückgabe: true = gültiger Messwert in 'cm'
#if SENSOR_TYP == SENSOR_4_20MA
float letzterStromMA = 0;

bool fuellstandMessen(float &cm) {
  // 16 Einzelmessungen mitteln; analogReadMilliVolts nutzt die Werkskalibrierung
  uint32_t summe = 0;
  for (int i = 0; i < 16; i++) summe += analogReadMilliVolts(PIN_SENSOR_ADC);
  const float mV = summe / 16.0f;
  const float mA = mV / SHUNT_OHM;          // U = R · I  →  I = U / R
  letzterStromMA = mA;

  if (mA < MA_FEHLER_UNTEN || mA > MA_FEHLER_OBEN) return false;

  cm = SENSOR_0_CM + (mA - 4.0f) / 16.0f * (SENSOR_20MA_CM - SENSOR_0_CM);
  cm = constrain(cm, 0.0f, TANK_HOEHE_CM);
  return true;
}

#elif SENSOR_TYP == SENSOR_ULTRASCHALL
// Rollender Median über die letzten 5 Messungen: eine Messung pro Takt,
// damit der Reglertakt nicht durch mehrere pulseIn() blockiert wird.
float usPuffer[5] = {0};
int   usIndex = 0, usAnzahl = 0, usFehlerFolge = 0;

static float median5(const float *w, int n) {
  float t[5];
  for (int i = 0; i < n; i++) t[i] = w[i];
  for (int i = 1; i < n; i++) {                 // Insertion-Sort, n ≤ 5
    float x = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > x) { t[j + 1] = t[j]; j--; }
    t[j + 1] = x;
  }
  return t[n / 2];
}

bool fuellstandMessen(float &cm) {
  digitalWrite(PIN_US_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH); delayMicroseconds(20);   // JSN-SR04T: ≥ 10 µs
  digitalWrite(PIN_US_TRIG, LOW);
  const uint32_t dauer = pulseIn(PIN_US_ECHO, HIGH, US_TIMEOUT_US);

  if (dauer == 0) {                              // kein Echo
    if (++usFehlerFolge >= 5) return false;      // erst nach 5 Fehlern in Folge
  } else {
    const float abstand = dauer * 0.0343f / 2.0f; // Schall ~343 m/s bei 20 °C
    if (abstand >= US_MIN_CM && abstand <= US_ABSTAND_LEER_CM + 10.0f) {
      usPuffer[usIndex] = abstand;
      usIndex = (usIndex + 1) % 5;
      if (usAnzahl < 5) usAnzahl++;
      usFehlerFolge = 0;
    } else if (++usFehlerFolge >= 5) {
      return false;
    }
  }
  if (usAnzahl == 0) return false;

  cm = US_ABSTAND_LEER_CM - median5(usPuffer, usAnzahl);
  cm = constrain(cm, 0.0f, TANK_HOEHE_CM);
  return true;
}
#endif

// ============================================================================
// 6) PARAMETER SPEICHERN / LADEN (NVS, bleibt nach Stromausfall erhalten)
// ============================================================================
void parameterLaden() {
  nvs.begin("tankpid", true);
  sollwert = nvs.getFloat("sp", STD_SOLLWERT_CM);
  pid.kp   = nvs.getFloat("kp", STD_KP);
  pid.ki   = nvs.getFloat("ki", STD_KI);
  pid.kd   = nvs.getFloat("kd", STD_KD);
  nvs.end();
}

void parameterSpeichern() {
  nvs.begin("tankpid", false);
  nvs.putFloat("sp", sollwert);
  nvs.putFloat("kp", pid.kp);
  nvs.putFloat("ki", pid.ki);
  nvs.putFloat("kd", pid.kd);
  nvs.end();
  Serial.println(F("Parameter gespeichert."));
}

// ============================================================================
// 7) SERIELLE BEDIENUNG
// ============================================================================
void hilfeZeigen() {
  Serial.println(F("\n--- TankPID Befehle ---"));
  Serial.println(F("  sp 83     Sollwert in cm"));
  Serial.println(F("  kp 10     P-Verstaerkung (%/cm)"));
  Serial.println(F("  ki 0.1    I-Verstaerkung (%/(cm*s))"));
  Serial.println(F("  kd 0      D-Verstaerkung (%*s/cm)"));
  Serial.println(F("  hand 30   Handbetrieb, Ventil fest auf 30 %"));
  Serial.println(F("  auto      Automatik (stossfreie Umschaltung)"));
  Serial.println(F("  save      Parameter dauerhaft speichern"));
  Serial.println(F("  plot      Ausgabe fuer seriellen Plotter an/aus"));
  Serial.println(F("  info      Status anzeigen"));
  Serial.println(F("  ?         diese Hilfe\n"));
}

void statusZeigen() {
  Serial.printf("Betrieb: %s | Soll %.1f cm | Ist %.2f cm | e %.2f cm | Ventil %.1f %%\n",
                betrieb == Betrieb::AUTO ? "AUTO" : "HAND",
                sollwert, istwert, sollwert - istwert, ventil);
  Serial.printf("Kp %.2f | Ki %.3f | Kd %.2f | P %.1f  I %.1f  D %.1f\n",
                pid.kp, pid.ki, pid.kd, pid.anteilP, pid.anteilI, pid.anteilD);
#if SENSOR_TYP == SENSOR_4_20MA
  Serial.printf("Sensorstrom: %.2f mA\n", letzterStromMA);
#endif
  if (sensorFehler) Serial.println(F("STOERUNG: Sensorfehler -> Zulauf ZU"));
  if (notHoch)      Serial.println(F("STOERUNG: Fuellstand zu hoch -> Zulauf ZU"));
}

void befehlAuswerten(String zeile) {
  zeile.trim();
  zeile.toLowerCase();
  if (zeile.length() == 0) return;

  const int leer = zeile.indexOf(' ');
  const String cmd  = leer < 0 ? zeile : zeile.substring(0, leer);
  const String arg  = leer < 0 ? ""    : zeile.substring(leer + 1);
  const float  wert = arg.toFloat();
  const bool   hatWert = arg.length() > 0;

  if (cmd == "sp" && hatWert) {
    sollwert = constrain(wert, 0.0f, NOT_HOCH_RESET_CM);
    Serial.printf("Sollwert = %.1f cm\n", sollwert);
  } else if (cmd == "kp" && hatWert && wert >= 0) {
    pid.kp = wert;  Serial.printf("Kp = %.2f\n", pid.kp);
  } else if (cmd == "ki" && hatWert && wert >= 0) {
    pid.ki = wert;  Serial.printf("Ki = %.3f\n", pid.ki);
  } else if (cmd == "kd" && hatWert && wert >= 0) {
    pid.kd = wert;  Serial.printf("Kd = %.2f\n", pid.kd);
  } else if (cmd == "hand" && hatWert) {
    handWert = constrain(wert, VENTIL_MIN_PROZENT, VENTIL_MAX_PROZENT);
    betrieb = Betrieb::HAND;
    Serial.printf("HAND: Ventil = %.1f %%\n", handWert);
  } else if (cmd == "auto") {
    pid.reset(istwert, ventil);       // stoßfrei: Integrator übernimmt Ventilwert
    betrieb = Betrieb::AUTO;
    Serial.println(F("AUTO"));
  } else if (cmd == "save") {
    parameterSpeichern();
  } else if (cmd == "plot") {
    plotAusgabe = !plotAusgabe;
    Serial.println(plotAusgabe ? F("Plot an") : F("Plot aus"));
  } else if (cmd == "info") {
    statusZeigen();
  } else if (cmd == "?" || cmd == "help" || cmd == "hilfe") {
    hilfeZeigen();
  } else {
    Serial.println(F("Unbekannter Befehl – '?' fuer Hilfe"));
  }
}

void seriellLesen() {
  static String puffer;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (puffer.length()) befehlAuswerten(puffer);
      puffer = "";
    } else if (puffer.length() < 40) {
      puffer += c;
    }
  }
}

// ============================================================================
// 8) SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_LED_STOERUNG, OUTPUT);
  digitalWrite(PIN_LED_STOERUNG, LOW);

  ventilInit();
  ventilSetzen(0);                    // sicherer Start: Zulauf ZU

#if SENSOR_TYP == SENSOR_4_20MA
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SENSOR_ADC, ADC_11db);   // Messbereich bis ~3,1 V
#elif SENSOR_TYP == SENSOR_ULTRASCHALL
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
#endif

  parameterLaden();
  pid.ausMin = VENTIL_MIN_PROZENT;
  pid.ausMax = VENTIL_MAX_PROZENT;
  pid.dTau   = D_FILTER_TAU_S;
  pid.reset(0, 0);

  Serial.println(F("\nTankPID gestartet"));
  hilfeZeigen();
  statusZeigen();

  letzterTakt = millis();
  letzterTaktUs = micros();
}

// ============================================================================
// 9) LOOP – fester Reglertakt, nicht blockierend
// ============================================================================
void loop() {
  seriellLesen();

  const uint32_t jetzt = millis();
  if (jetzt - letzterTakt < ABTASTZEIT_MS) return;
  letzterTakt += ABTASTZEIT_MS;
  if (jetzt - letzterTakt > ABTASTZEIT_MS) letzterTakt = jetzt;  // nach Hängern nicht nachholen

  const uint32_t us = micros();
  const float dt = (us - letzterTaktUs) / 1e6f;   // echte Zeit seit letztem Takt
  letzterTaktUs = us;

  // --- Messen ----------------------------------------------------------------
  float roh;
  const bool ok = fuellstandMessen(roh);
  sensorFehler = !ok;
  if (ok) {
    if (!istInit) {                               // erster Wert: Filter vorbelegen
      istwert = roh;
      pid.reset(istwert, ventil);
      istInit = true;
    }
    const float a = dt / (MESS_FILTER_S + dt);    // Tiefpass 1. Ordnung
    istwert += a * (roh - istwert);
  }

  // --- Überlaufschutz mit Hysterese -----------------------------------------
  if (istwert >= NOT_HOCH_CM)            notHoch = true;
  else if (istwert <= NOT_HOCH_RESET_CM) notHoch = false;

  // --- Regeln ----------------------------------------------------------------
  if (sensorFehler || notHoch || !istInit) {
    ventil = 0;                                   // Fail-safe: Zulauf ZU
    pid.reset(istwert, 0);                        // kein Windup während Störung
  } else if (betrieb == Betrieb::HAND) {
    ventil = handWert;
    pid.reset(istwert, ventil);                   // Regler "läuft mit"
  } else {
    ventil = pid.berechne(sollwert, istwert, dt);
  }
  ventilSetzen(ventil);
  digitalWrite(PIN_LED_STOERUNG, (sensorFehler || notHoch) ? HIGH : LOW);

  // --- Ausgabe (alle 500 ms) ------------------------------------------------
  if (plotAusgabe && jetzt - letzteAusgabe >= 500) {
    letzteAusgabe = jetzt;
    // Format für den seriellen Plotter der Arduino IDE 2.x
    Serial.printf("Soll:%.2f,Ist:%.2f,Ventil:%.1f,P:%.1f,I:%.1f,D:%.1f\n",
                  sollwert, istwert, ventil, pid.anteilP, pid.anteilI, pid.anteilD);
  }
}
