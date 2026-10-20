/*
  ==========================================================================
  PID_Temperatur.ino  –  Temperaturregelung mit ESP32 (Arduino IDE)
  ==========================================================================

  Funktion
  --------
  - Istwert:   DS18B20 (OneWire), nicht-blockierend gelesen
  - Stellglied: Heizung über SSR (Solid-State-Relais), angesteuert mit
                Zeit-Proportional-PWM (Schwingungspaket-Prinzip):
                Stellwert 0..100 % = Einschaltanteil innerhalb eines Zeitfensters
  - Regler:    PID mit Anti-Windup, D-Anteil auf den Istwert (kein D-Kick)
  - Sicherheit: Übertemperatur-Abschaltung und Sensorfehler -> Heizung AUS,
                Zustand verriegelt bis Quittierung ("r")
  - Bedienung: Serieller Monitor (115200 Baud), Parameter im Flash speicherbar
  - Ausgabe:   CSV-Zeile pro Abtastung -> direkt im Seriellen Plotter nutzbar

  Benötigte Bibliotheken (Bibliotheksverwalter)
  ---------------------------------------------
  - OneWire            (Paul Stoffregen)
  - DallasTemperature  (Miles Burton)
  - Preferences        (im ESP32-Core enthalten)

  Verdrahtung
  -----------
  DS18B20:  VDD -> 3,3 V, GND -> GND, DATA -> GPIO 4  + 4,7 kΩ Pull-up nach 3,3 V
  SSR:      Steuer+ -> GPIO 26, Steuer- -> GND  (SSR mit 3-32 V DC Eingang)
  LED:      Onboard-LED GPIO 2 (zeigt Heizung an, blinkt schnell bei Fehler)

  Serielle Befehle (jeweils mit Enter abschließen)
  ------------------------------------------------
  s <wert>   Sollwert in °C            z.B.  s 60
  p <wert>   Kp setzen                  z.B.  p 8
  i <wert>   Ki setzen (1/s)            z.B.  i 0.05
  d <wert>   Kd setzen (s)              z.B.  d 20
  m <wert>   Handbetrieb, Stellwert %   z.B.  m 30
  a          Automatikbetrieb (PID)
  r          Fehler quittieren
  w          Parameter im Flash speichern
  ?          Status anzeigen
  ==========================================================================
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>

// ---------------------------- Konfiguration -------------------------------
constexpr uint8_t  PIN_ONEWIRE     = 4;
constexpr uint8_t  PIN_HEIZUNG     = 26;
constexpr uint8_t  PIN_LED         = 2;

constexpr uint32_t ABTASTZEIT_MS   = 1000;   // Reglertakt (DS18B20 12 bit braucht 750 ms)
constexpr uint32_t PWM_FENSTER_MS  = 2000;   // Zeitfenster für SSR-Taktung
constexpr uint32_t MIN_SCHALT_MS   = 50;     // kürzeste Ein-/Auszeit (SSR schonen)

constexpr float    T_MAX           = 95.0f;  // Übertemperatur -> Abschaltung
constexpr float    SOLL_MIN        = 0.0f;
constexpr float    SOLL_MAX        = 90.0f;

// ---------------------------- PID-Baustein --------------------------------
struct PID {
  float kp = 8.0f, ki = 0.05f, kd = 20.0f;
  float outMin = 0.0f, outMax = 100.0f;
  float dtMax  = 5.0f;          // längere Pausen -> Neustart des Reglers

  float iAnteil    = 0.0f;      // fertig gewichteter I-Anteil (ki bereits enthalten)
  float prevInput  = 0.0f;
  uint32_t lastMicros = 0;
  bool  neustart   = true;

  void reset() {
    iAnteil  = 0.0f;
    neustart = true;
  }

  // Stoßfreie Umschaltung Hand -> Automatik: I-Anteil auf aktuellen Stellwert setzen
  void uebernehmen(float stellwert, float istwert) {
    iAnteil    = constrain(stellwert, outMin, outMax);
    prevInput  = istwert;
    lastMicros = micros();
    neustart   = false;
  }

  float update(float sollwert, float istwert) {
    uint32_t jetzt = micros();
    float dt = (jetzt - lastMicros) / 1e6f;   // Sekunden (Überlauf-sicher durch uint32)
    lastMicros = jetzt;

    if (neustart || dt <= 0.0f || dt > dtMax) {   // erster Aufruf / Ausreißer
      prevInput = istwert;
      neustart  = false;
      return constrain(iAnteil, outMin, outMax);
    }

    float fehler = sollwert - istwert;

    // P-Anteil
    float pAnteil = kp * fehler;

    // D-Anteil auf den Istwert -> kein Sprung bei Sollwertänderung
    float dAnteil = -kd * (istwert - prevInput) / dt;
    prevInput = istwert;

    // I-Anteil mit Anti-Windup (bedingte Integration + Begrenzung)
    float iNeu = iAnteil + ki * fehler * dt;
    float outTest = pAnteil + iNeu + dAnteil;
    bool  saettigtOben  = (outTest > outMax) && (fehler > 0);
    bool  saettigtUnten = (outTest < outMin) && (fehler < 0);
    if (!saettigtOben && !saettigtUnten) {
      iAnteil = constrain(iNeu, outMin, outMax);
    }

    float out = pAnteil + iAnteil + dAnteil;
    return constrain(out, outMin, outMax);
  }
};

// ---------------------------- Globale Objekte -----------------------------
OneWire           oneWire(PIN_ONEWIRE);
DallasTemperature sensoren(&oneWire);
Preferences       prefs;
PID               pid;

enum class Modus : uint8_t { AUTO, HAND, FEHLER };
Modus  modus        = Modus::AUTO;

float  sollwert     = 50.0f;
float  istwert      = NAN;
float  stellwert    = 0.0f;   // 0..100 %
float  handwert     = 0.0f;
String fehlerText   = "";

uint32_t letzteAbtastung = 0;
uint32_t fensterStart    = 0;
bool     messungLaeuft   = false;
String   eingabe;

// ---------------------------- Parameter im Flash --------------------------
void parameterLaden() {
  prefs.begin("pidtemp", true);
  sollwert = prefs.getFloat("soll", sollwert);
  pid.kp   = prefs.getFloat("kp", pid.kp);
  pid.ki   = prefs.getFloat("ki", pid.ki);
  pid.kd   = prefs.getFloat("kd", pid.kd);
  prefs.end();
}

void parameterSpeichern() {
  prefs.begin("pidtemp", false);
  prefs.putFloat("soll", sollwert);
  prefs.putFloat("kp", pid.kp);
  prefs.putFloat("ki", pid.ki);
  prefs.putFloat("kd", pid.kd);
  prefs.end();
  Serial.println(F("# Parameter gespeichert"));
}

// ---------------------------- Hilfsfunktionen -----------------------------
void heizungAus() {
  digitalWrite(PIN_HEIZUNG, LOW);
  stellwert = 0.0f;
}

void fehlerSetzen(const String& text) {
  if (modus != Modus::FEHLER) {
    modus      = Modus::FEHLER;
    fehlerText = text;
    heizungAus();
    Serial.print(F("# FEHLER: "));
    Serial.println(text);
  }
}

void statusAusgeben() {
  Serial.println(F("# ---------- Status ----------"));
  Serial.printf("# Modus:     %s\n",
                modus == Modus::AUTO ? "AUTOMATIK" :
                modus == Modus::HAND ? "HAND" : "FEHLER");
  if (modus == Modus::FEHLER) Serial.printf("# Fehler:    %s\n", fehlerText.c_str());
  Serial.printf("# Sollwert:  %.2f °C\n", sollwert);
  Serial.printf("# Istwert:   %.2f °C\n", istwert);
  Serial.printf("# Stellwert: %.1f %%\n", stellwert);
  Serial.printf("# Kp=%.3f  Ki=%.4f  Kd=%.3f\n", pid.kp, pid.ki, pid.kd);
  Serial.println(F("# ----------------------------"));
}

// ---------------------------- Serielle Befehle ----------------------------
void befehlAusfuehren(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char  c    = tolower(cmd.charAt(0));
  float wert = cmd.substring(1).toFloat();

  switch (c) {
    case 's':
      sollwert = constrain(wert, SOLL_MIN, SOLL_MAX);
      Serial.printf("# Sollwert = %.2f °C\n", sollwert);
      break;
    case 'p': pid.kp = max(0.0f, wert); Serial.printf("# Kp = %.3f\n", pid.kp); break;
    case 'i': pid.ki = max(0.0f, wert); Serial.printf("# Ki = %.4f\n", pid.ki); break;
    case 'd': pid.kd = max(0.0f, wert); Serial.printf("# Kd = %.3f\n", pid.kd); break;
    case 'm':
      if (modus == Modus::FEHLER) { Serial.println(F("# Erst Fehler quittieren (r)")); break; }
      handwert = constrain(wert, 0.0f, 100.0f);
      modus    = Modus::HAND;
      Serial.printf("# Handbetrieb, Stellwert = %.1f %%\n", handwert);
      break;
    case 'a':
      if (modus == Modus::FEHLER) { Serial.println(F("# Erst Fehler quittieren (r)")); break; }
      if (modus == Modus::HAND && !isnan(istwert)) pid.uebernehmen(handwert, istwert);
      modus = Modus::AUTO;
      Serial.println(F("# Automatikbetrieb"));
      break;
    case 'r':
      if (modus == Modus::FEHLER) {
        modus = Modus::AUTO;
        fehlerText = "";
        pid.reset();
        Serial.println(F("# Fehler quittiert, Automatikbetrieb"));
      }
      break;
    case 'w': parameterSpeichern(); break;
    case '?': statusAusgeben(); break;
    default:  Serial.println(F("# Unbekannter Befehl (? fuer Status)")); break;
  }
}

void seriellLesen() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      befehlAusfuehren(eingabe);
      eingabe = "";
    } else if (eingabe.length() < 32) {
      eingabe += ch;
    }
  }
}

// ---------------------------- Messung + Regelung --------------------------
void abtastung() {
  // 1) Ergebnis der im letzten Zyklus gestarteten Messung abholen
  if (messungLaeuft) {
    float t = sensoren.getTempCByIndex(0);

    if (t == DEVICE_DISCONNECTED_C) {
      fehlerSetzen("Sensor nicht gefunden / getrennt");
    } else if (t == 85.0f && isnan(istwert)) {
      // 85 °C ist der Einschaltwert des DS18B20 -> erste Messung verwerfen
    } else {
      istwert = t;
      if (istwert >= T_MAX) fehlerSetzen("Uebertemperatur");
    }
  }

  // 2) Neue Messung starten (nicht blockierend)
  sensoren.requestTemperatures();
  messungLaeuft = true;

  // 3) Stellwert berechnen
  if (isnan(istwert) || modus == Modus::FEHLER) {
    heizungAus();
  } else if (modus == Modus::HAND) {
    stellwert = handwert;
    pid.uebernehmen(handwert, istwert);   // Regler läuft mit -> stoßfrei zurück
  } else {
    stellwert = pid.update(sollwert, istwert);
  }

  // 4) CSV für Seriellen Plotter
  Serial.printf("Soll:%.2f,Ist:%.2f,Stell:%.1f\n",
                sollwert, isnan(istwert) ? 0.0f : istwert, stellwert);
}

// ---------------------------- SSR-Ansteuerung -----------------------------
void ssrAnsteuern() {
  uint32_t jetzt = millis();
  if (jetzt - fensterStart >= PWM_FENSTER_MS) fensterStart += PWM_FENSTER_MS;

  if (modus == Modus::FEHLER) {
    digitalWrite(PIN_HEIZUNG, LOW);
    return;
  }

  uint32_t einZeit = (uint32_t)(stellwert / 100.0f * PWM_FENSTER_MS);
  if (einZeit < MIN_SCHALT_MS) einZeit = 0;                               // zu kurz -> aus
  if (einZeit > PWM_FENSTER_MS - MIN_SCHALT_MS) einZeit = PWM_FENSTER_MS; // fast voll -> an

  digitalWrite(PIN_HEIZUNG, (jetzt - fensterStart) < einZeit ? HIGH : LOW);
}

void ledAnzeigen() {
  if (modus == Modus::FEHLER) {
    digitalWrite(PIN_LED, (millis() / 150) % 2);   // schnelles Blinken
  } else {
    digitalWrite(PIN_LED, digitalRead(PIN_HEIZUNG));
  }
}

// ---------------------------- Setup / Loop --------------------------------
void setup() {
  pinMode(PIN_HEIZUNG, OUTPUT);
  digitalWrite(PIN_HEIZUNG, LOW);   // sofort sicherer Zustand
  pinMode(PIN_LED, OUTPUT);

  Serial.begin(115200);
  delay(200);
  Serial.println(F("# PID-Temperaturregelung ESP32"));

  sensoren.begin();
  sensoren.setResolution(12);
  sensoren.setWaitForConversion(false);   // nicht blockierend
  if (sensoren.getDeviceCount() == 0) {
    fehlerSetzen("Kein DS18B20 gefunden");
  }

  parameterLaden();
  pid.outMin = 0.0f;
  pid.outMax = 100.0f;
  pid.dtMax  = 5.0f * ABTASTZEIT_MS / 1000.0f;
  pid.reset();

  statusAusgeben();
  letzteAbtastung = millis();
  fensterStart    = millis();
}

void loop() {
  seriellLesen();

  if (millis() - letzteAbtastung >= ABTASTZEIT_MS) {
    letzteAbtastung += ABTASTZEIT_MS;
    abtastung();
  }

  ssrAnsteuern();
  ledAnzeigen();
}
