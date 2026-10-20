# Technische Projektdokumentation & Handbuch
## Autonome PID-Füllstandsregelung mit ESP32 Mikrocontroller

**Dokument-ID:** DOK-ESP32-PID-TANK-2026  
**Version:** 1.4.0  
**Status:** Inbetriebnahmereif / Freigegeben  
**Erstellungsdatum:** 20.10.2026  
**Zielplattform:** ESP32 DevKit V1 (Tensilica Xtensa Dual-Core 240 MHz)  
**Entwicklungsframework:** Arduino Core / ESP-IDF / FreeRTOS  

---

## Inhaltsverzeichnis
1. [Management Summary & Systemübersicht](#1-management-summary--systemübersicht)
2. [Mathematisch-physikalisches Streckenmodell](#2-mathematisch-physikalisches-streckenmodell)
3. [Entwurf des diskreten PID-Reglers](#3-entwurf-des-diskreten-pid-reglers)
4. [Anti-Windup & Derivative-on-Measurement](#4-anti-windup--derivative-on-measurement)
5. [Hardware-Architektur & Schaltplan](#5-hardware-architektur--schaltplan)
6. [Software-Architektur & FreeRTOS-Tasking](#6-software-architektur--freertos-tasking)
7. [Inbetriebnahme- & Tuning-Leitfaden](#7-inbetriebnahme---tuning-leitfaden)
8. [Fehlerdiagnose & Troubleshooting](#8-fehlerdiagnose--troubleshooting)
9. [Anlagen-Spezifikation (Aktuelle Konfigurationsparameter)](#9-anlagen-spezifikation-aktuelle-konfigurationsparameter)
10. [Anhang: Schnelle Checkliste für die Werkbank](#10-anhang-schnelle-checkliste-für-die-werkbank)

---

## 1. Management Summary & Systemübersicht

Die vorliegende Dokumentation beschreibt die Konzeption, Modellbildung, elektronische Schaltungsauslegung und softwaretechnische Implementierung einer industrietauglichen Füllstandsregelung für einen zylindrischen Flüssigkeitsbehälter. 

Als Steuerungsrechner dient ein **ESP32-Mikrocontroller**, welcher aufgrund seiner Dual-Core-Architektur (Core 0 für Kommunikation/Web, Core 1 für zeitkritische Regelungsalgorithmen), der integrierten LEDC-Hardware-PWM-Timer und des nativen FreeRTOS-Echtzeitbetriebssystems eine robuste, deterministische Plattform darstellt.

### Kernaufgaben des Systems:
* **Führungsverhalten:** Präzises Anfahren und Halten des Sollwerts ($w = 60.0\text{ cm}$) ohne unzulässiges Überschwingen.
* **Störgrößenkompensation:** Schnelle Ausregelung von unkontrollierten Flüssigkeitsentnahmen über ein variierendes Bodenventil (Torricelli-Ablauf).
* **Aktorschonung:** Vermeidung von hochfrequentem Takten und Schwingungen durch differenzierte Dämpfung, Totbandregelung und Ventil-Begrenzung.

---

## 2. Mathematisch-physikalisches Streckenmodell

### 2.1 Kontinuitätsgleichung (Massen- und Volumenbilanz)
Der Füllstand $h(t)$ im Behälter wird durch die zeitliche Änderung des gespeicherten Flüssigkeitsvolumens $V(t)$ bestimmt:

$$\frac{dV(t)}{dt} = Q_{in}(t) - Q_{out}(t)$$

Für einen stehenden zylindrischen Behälter mit Innendurchmesser $D = 50.0\text{ cm}$ beträgt die konstante Querschnittsfläche:

$$A = \pi \cdot \left(\frac{D}{2}\right)^2 = \pi \cdot \left(\frac{50.0\text{ cm}}{2}\right)^2 \approx 1963.5\text{ cm}^2$$

Daraus folgt die fundamentale Differentialgleichung der Regelstrecke:

$$A \cdot \frac{dh(t)}{dt} = Q_{in}(t) - Q_{out}(t) \quad \Longleftrightarrow \quad \frac{dh(t)}{dt} = \frac{1}{A} \cdot \Big( Q_{in}(t) - Q_{out}(t) \Big)$$

### 2.2 Nichtlinearer Ablauf nach dem Torricelli-Gesetz
Im Gegensatz zum linearen Zulauf strömt die Flüssigkeit unter Schwerkraftwirkung $g \approx 9{,}81\text{ m/s}^2$ durch das Ablaufventil am Behälterboden aus. Nach Torricelli gilt für die Ausströmgeschwindigkeit $v = \sqrt{2 \cdot g \cdot h(t)}$.

Der resultierende Volumenstrom des Ablaufs ist:

$$Q_{out}(h) = \mu \cdot A_{valve} \cdot \sqrt{2 \cdot g \cdot h(t)}$$

* $\mu$: Ausflussziffer (Kontraktions- und Reibungskoeffizient, typisch $0{,}60 \dots 0{,}75$)
* $A_{valve}$: Freier Querschnitt des geöffneten Ablaufventils

**Regelungstechnische Konsequenz:**
Die Regelstrecke besitzt eine **nichtlineare Kennlinie**. Bei hohem Füllstand ist der hydrostatische Druck hoch und der Ablauf maximal; bei niedrigem Füllstand sinkt der Ablauf drastisch. Die differentielle Streckenverstärkung $K_S(h) = \frac{\partial h}{\partial Q}$ ist daher arbeitspunktabhängig. Ein rein proportionaler Regler kann den Füllstand bei vorhandenem Ablauf konstruktionsbedingt niemals stationär auf den Sollwert führen!

### 2.3 Dynamisches Aktorverhalten ($PT_1$-Glied)
Proportionalventile und DC-Pumpen reagieren mit mechanischer und hydraulischer Verzögerung. Dies wird durch ein Verzögerungsglied 1. Ordnung modelliert:

$$T_{valve} \cdot \frac{dQ_{in}(t)}{dt} + Q_{in}(t) = K_{valve} \cdot u(t)$$

mit der Ventilzeitkonstante $T_{valve} \approx 0.30\text{ s}$.

---

## 3. Entwurf des diskreten PID-Reglers

### 3.1 Kontinuierliche Übertragungsfunktion
Der ideale Eingrößen-PID-Regler im Frequenzbereich lautet:

$$G_R(s) = K_p + \frac{K_i}{s} + K_d \cdot s$$

mit Regeldifferenz $e(t) = w(t) - y(t)$ (Sollwert minus gemessener Istwert).

### 3.2 Zeitdiskrete Realisierung auf dem ESP32
Auf dem Mikrocontroller wird der Regler zyklisch mit der festen Abtastzeit $T_a = \Delta t = 100\text{ ms}$ aufgerufen:

#### Proportional-Anteil:
$$P_k = K_p \cdot e_k$$

#### Integral-Anteil (Trapez- / Euler-Integration):
$$I_k = I_{k-1} + K_i \cdot e_k \cdot \Delta t$$
*Der I-Anteil garantiert stationäre Genauigkeit ($e_\infty = 0$), da er den permanenten Torricelli-Ablaufverlust exakt ausgleicht.*

#### Differenzial-Anteil:
$$D_k = K_d \cdot \frac{e_k - e_{k-1}}{\Delta t}$$

### 3.3 Grundsätze der Integralrechnung einfach erklärt & Rolle im PID-Regler

#### A. Was bedeutet „Integrieren“ im Alltag?
Integrieren bedeutet schlichtweg: **kontinuierliches Aufsummieren vieler kleiner Abschnitte zu einer Gesamtsumme**.
* **Beispiel 1 (Auto):** Der Tachometer zeigt die Momentangeschwindigkeit $v(t) = \frac{ds}{dt}$ (die *Ableitung*). Der Kilometerzähler ist das *Integral*: Er summiert in jeder Sekunde den winzigen zurückgelegten Weg auf: $s = \int v(t) dt$.
* **Beispiel 2 (Wassertank):** Fließt Wasser mit Zuflussrate $Q(t)$ [L/s], ist das Gesamtvolumen im Tank nach 10 Sekunden das Integral aller Tropfen: $V = \int_0^{10} Q(t) dt$.

#### B. Geometrische Bedeutung: Die Fläche unter der Kurve (Rechteck-Integration über dt)
Grafisch ist das Integral der **Flächeninhalt zwischen der Fehlerkurve $e(t)$ und der Nulllinie**. Da der ESP32 nicht kontinuierlich rechnen kann, zerlegt er die Zeit in diskrete Zeitschritte $\Delta t = 100\text{ ms}$. In jedem Takt addiert er die Fläche eines Rechtecks ($e_k \cdot \Delta t$):
$$\int_0^t e(\tau) d\tau \approx \sum_{i=1}^k e_i \cdot \Delta t$$

**Grafische Darstellung der Euler- / Riemann-Integration:**

<div align="center">
<svg viewBox="0 0 640 240" width="100%" style="max-width: 640px; background: #0f172a; border-radius: 8px;" xmlns="http://www.w3.org/2000/svg">
  <line x1="50" y1="20" x2="50" y2="200" stroke="#334155" stroke-width="1.5" />
  <line x1="50" y1="180" x2="620" y2="180" stroke="#334155" stroke-width="1.5" />
  <text x="35" y="30" fill="#94a3b8" font-size="11" font-family="sans-serif">e(t) [cm]</text>
  <text x="580" y="200" fill="#94a3b8" font-size="11" font-family="sans-serif">Zeit t [s]</text>
  <text x="50" y="196" fill="#64748b" font-size="10" font-family="monospace">0</text>
  
  <!-- Riemann Rectangles -->
  <rect x="70" y="60" width="35" height="120" fill="#0284c7" fill-opacity="0.35" stroke="#38bdf8" stroke-width="1" />
  <rect x="105" y="75" width="35" height="105" fill="#0284c7" fill-opacity="0.35" stroke="#38bdf8" stroke-width="1" />
  <rect x="140" y="95" width="35" height="85" fill="#0284c7" fill-opacity="0.35" stroke="#38bdf8" stroke-width="1" />
  <rect x="175" y="120" width="35" height="60" fill="#0284c7" fill-opacity="0.35" stroke="#38bdf8" stroke-width="1" />
  <rect x="210" y="145" width="35" height="35" fill="#0284c7" fill-opacity="0.35" stroke="#38bdf8" stroke-width="1" />
  <rect x="245" y="165" width="35" height="15" fill="#0284c7" fill-opacity="0.35" stroke="#38bdf8" stroke-width="1" />
  
  <!-- Steady State -->
  <rect x="280" y="179" width="35" height="2" fill="#10b981" stroke="#10b981" stroke-width="1" />
  <rect x="315" y="179" width="35" height="2" fill="#10b981" stroke="#10b981" stroke-width="1" />

  <!-- Negative Area -->
  <rect x="350" y="180" width="35" height="25" fill="#ef4444" fill-opacity="0.35" stroke="#f87171" stroke-width="1" />
  <rect x="385" y="180" width="35" height="15" fill="#ef4444" fill-opacity="0.35" stroke="#f87171" stroke-width="1" />
  <rect x="420" y="179" width="35" height="2" fill="#10b981" stroke="#10b981" stroke-width="1" />

  <!-- Continuous Curve -->
  <path d="M 50,45 C 80,50 120,80 170,115 C 220,150 260,175 300,180 C 340,185 360,205 385,200 C 410,195 430,180 500,180 L 600,180" fill="none" stroke="#38bdf8" stroke-width="2.5" />

  <!-- dt Annotation -->
  <line x1="70" y1="188" x2="105" y2="188" stroke="#38bdf8" stroke-width="1.5" />
  <line x1="70" y1="185" x2="70" y2="191" stroke="#38bdf8" stroke-width="1.5" />
  <line x1="105" y1="185" x2="105" y2="191" stroke="#38bdf8" stroke-width="1.5" />
  <text x="88" y="202" fill="#38bdf8" font-size="10" font-family="monospace" text-anchor="middle">dt</text>

  <text x="130" y="55" fill="#38bdf8" font-size="11" font-weight="bold">Fehler e(t) &gt; 0</text>
  <text x="110" y="140" fill="#e0f2fe" font-size="10" font-family="monospace">Fläche = e_k · dt</text>
  <text x="100" y="155" fill="#93c5fd" font-size="9">(Integral wächst ↑)</text>

  <text x="280" y="160" fill="#34d399" font-size="10" font-weight="bold">e = 0 (Sollwert erreicht)</text>
  <text x="280" y="172" fill="#a7f3d0" font-size="9">(Integral behält Wert bei!)</text>

  <text x="360" y="225" fill="#f87171" font-size="10">e &lt; 0: Negativ</text>
  <text x="360" y="235" fill="#fca5a5" font-size="9">(Integral schrumpft ↓)</text>
</svg>
</div>

*Legende: Jeder Zeitschritt dt fügt ein Rechteck der Höhe e(t) und Breite dt zum Integratorspeicher hinzu (`iTerm += Ki * error * dt`).*

#### C. Warum ist das Integral das unverzichtbare „Langzeitgedächtnis“ des Reglers?
Ein reiner P-Regler berechnet nur $P = K_p \cdot e(t)$. Ist der Sollwert erreicht ($e = 0$), schließt der P-Regler das Ventil sofort ($P = 0$). Da aber Wasser durch den Boden abfließt, fällt der Pegel sofort wieder ab (Offset / bleibende Regelabweichung).

Hier greift der I-Anteil als **Gedächtnis**:
1. **Solange $e(t) > 0$:** Es kommt mit jedem Takt positive Fläche hinzu. Der Wert $I_k$ wächst stetig weiter an $\rightarrow$ das Ventil öffnet immer weiter.
2. **Wenn $e(t) = 0$ erreicht ist:** Die neu hinzukommende Fläche ist $0 \cdot \Delta t = 0$. **Das Integral bleibt exakt auf seinem letzten Summenwert stehen!** Es hält das Ventil dauerhaft z.B. bei $38\%$ geöffnet – genau so viel, wie durch den Ablauf entweicht.
3. **Wenn $e(t) < 0$ (Tank zu voll):** Die Fläche wird negativ $\rightarrow$ das Integral baut sich ab und das Ventil schließt.

---

## 4. Anti-Windup & Derivative-on-Measurement

### 4.1 Anti-Windup (Clamping-Verfahren)
Wenn das Stellventil an seine physikalische Grenze stößt ($u = 0\%$ voll geschlossen oder $u = 100\%$ voll geöffnet), die Regeldifferenz $e$ jedoch weiterhin ungleich null ist (z.B. beim initialen Befüllen eines leeren Tanks), würde ein Standard-Integrator unbegrenzt weiter integrieren (**Integrator Windup**). 

Sobald der Sollwert erreicht wird, müsste der riesige I-Wert erst mühsam abgebaut werden, was zu einem **massiven Überschwingen** führt.

**Implementiertes Clamping:**
Der Integrationsschritt wird nur dann ausgeführt, wenn das Stellsignal nicht in der Sättigung ist ODER wenn das Vorzeichen des neuen Fehlers den Integrator aus der Sättigung zurückholt:
```cpp
float unconstrainedOutput = P + (iTerm + Ki * error * dt) + D;
if (unconstrainedOutput >= 0.0f && unconstrainedOutput <= 100.0f) {
    iTerm += Ki * error * dt; // Nur im linearen Bereich akkumulieren
}
```

### 4.2 Derivative-on-Measurement (Vermeidung des "Derivative Kick")
Wird der Sollwert sprunghaft verstellt (z.B. von 30 cm auf 80 cm), ist $\frac{de}{dt} = \frac{dw - dy}{dt}$ im ersten Takt theoretisch unendlich. Der Regler würde einen gewaltigen Strom- bzw. Druckstoß auf das Ventil abgeben (**Derivative Kick**).

Daher wird die Ableitung **nur auf den gemessenen Prozesswert $y$** angewendet:

$$D_k = -K_d \cdot \frac{y_k - y_{k-1}}{\Delta t}$$

Dies dämpft schnelle Bewegungen der Wasseroberfläche optimal, ohne bei Sollwertsprüngen in das System zu schlagen.

### 4.3 Totband (Deadband)
Kleine Schwankungen der Wasseroberfläche (z.B. Wellen oder Sensorrauschen $\pm 0.30\text{ cm}$) führen ohne Totband zu ständiger Mikro-Bewegung des Ventils:
```cpp
if (abs(error) < DEADBAND_CM) {
    error = 0.0f; // Ventil bleibt ruhig
}
```

---

## 5. Hardware-Architektur & Schaltplan

### 5.1 ESP32 Pinbelegung

| Signal / Funktion | ESP32 GPIO | Peripherie / Kanal | Bemerkung |
| :--- | :--- | :--- | :--- |
| **Sensor Trigger** | **GPIO 5** | Digital Out / I2C | 10µs Ansteuerpuls |
| **Sensor Echo** | **GPIO 18** | Digital In (Spannungsteiler!) | **ACHTUNG: 5V Echo muss über 2.2kΩ/4.7kΩ auf 3.3V geteilt werden!** |
| **Aktor PWM** | **GPIO 25** | **LEDC Channel 0, Timer 0** | 5000 Hz, 10 Bit (1024 Stufen) |
| **Status-LED** | GPIO 2 | Integrierte Board-LED | Blinkt im Takt der Regelungs-Schleife |

### 5.2 Leistungsstufe & Schutzbeschaltung
Industrielle Magnetventile oder 12V/24V Pumpen dürfen **niemals direkt** an den ESP32 angeschlossen werden (Gefahr der Zerstörung des Prozessors):
1. **MOSFET:** Logic-Level N-Kanal MOSFET (z.B. IRLZ44N oder IRLB8721 mit $V_{GS(th)} < 2{,}5\text{V}$).
2. **Gate-Widerstände:** 
   * $220\,\Omega$ Serienwiderstand am Gate (begrenzt kapazitiven Umladestrom des ESP32 Pins).
   * $10\,\text{k}\Omega$ Pull-Down Widerstand zwischen Gate und GND (garantiert sicheres Abschalten beim Booten).
3. **Freilaufdiode:** Eine Schottky-Diode (z.B. 1N5819) oder Standarddiode (1N4007) **antiparallel** zur induktiven Last (Ventilspule / Motor). Sie schluckt die beim Abschalten entstehende Gegeninduktionsspannung von mehreren hundert Volt.
4. **Galvanische Trennung (Optional für Industrieumgebungen):** Optokoppler PC817 zwischen ESP32 GPIO und MOSFET-Treiber.

---

## 6. Software-Architektur & FreeRTOS-Tasking

Die Firmware verwendet das in ESP-IDF / Arduino enthaltene FreeRTOS. Der Regelungs-Task wird mit hoher Priorität auf **Core 1** isoliert, während Hintergrund-Webserver, WLAN und Serial-Logging auf **Core 0** laufen.

```cpp
void vPidControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Exakter Takt

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Jitterfreies Timing

        // 1. Messung einlesen (Sensor)
        float currentLevel = readSensorLevelCm();

        // 2. PID-Algorithmus ausführen
        float controlOutput = computePid(currentLevel, SETPOINT_CM);

        // 3. Stellsignal auf Hardware ausgeben (LEDC PWM)
        applyActuatorOutput(controlOutput);
    }
}
```

---

## 7. Inbetriebnahme- & Tuning-Leitfaden

### Schritt 1: Sensor-Nullpunkt & Kalibrierung
1. Den leeren Tank mit Wasser bis zum Boden des Sensors füllen ($h = 0\text{ cm}$).
2. Raw-Werte über den Seriellen Monitor prüfen.
3. Bei Ultraschall beachten: Der Mindestabstand zum Geber beträgt beim JSN-SR04T ca. 20 cm. Der Sensor muss oberhalb des Maximalpegels montiert sein!

### Schritt 2: Einstellregeln nach Ziegler-Nichols (Schwingungsmethode)
1. Integrator und Differentiator deaktivieren: $K_i = 0$, $K_d = 0$.
2. Den Proportionalbeiwert $K_p$ von 0 beginnend langsam schrittweise erhöhen.
3. Einen kleinen Sollwertsprung vorgeben ($+10\text{ cm}$).
4. Sobald der Füllstand in eine **gleichförmige, ungedämpfte Dauerschwingung** gerät, die Werte notieren:
   * **Kritisches $K_{p,crit}$**
   * **Periodendauer der Schwingung $T_{crit}$ (in Sekunden)**
5. Reglerparameter nach Tabelle berechnen:

| Reglertyp | $K_p$ | $T_n$ (Integrationszeit) | $T_v$ (Vorhaltezeit) |
| :--- | :--- | :--- | :--- |
| **P-Regler** | $0{,}50 \cdot K_{crit}$ | - | - |
| **PI-Regler** | $0{,}45 \cdot K_{crit}$ | $0{,}83 \cdot T_{crit}$ | - |
| **Klassischer PID** | $0{,}60 \cdot K_{crit}$ | $0{,}50 \cdot T_{crit}$ | $0{,}125 \cdot T_{crit}$ |
| **PID ohne Überschwingen (CHR)** | $0{,}40 \cdot K_{crit}$ | $0{,}80 \cdot T_{crit}$ | $0{,}150 \cdot T_{crit}$ |

*Hinweis: Umrechnung auf Software-Parameter: $K_i = \frac{K_p}{T_n}$, $K_d = K_p \cdot T_v$.*

### Schritt 3: Praxisnahes Feintuning am Prüfstand
* **Kriechende Annäherung / zu träge:** $K_p$ um 15% anheben.
* **Dauerhafte Abweichung unter Sollwert:** $K_i$ moderat erhöhen (z.B. in Schritten von $+0{,}02$).
* **Heftiges Schwingen um den Sollwert:** $K_p$ um 30% senken, $K_d$ leicht anheben zur Dämpfung.
* **Ventil flattert unruhig:** Totband erhöhen (z.B. von 0.2 auf 0.5 cm) oder Sensorfilter-Zeitkonstante erhöhen.

---

## 8. Fehlerdiagnose & Troubleshooting

| Symptom | Mögliche Ursache | Abhilfemaßnahme |
| :--- | :--- | :--- |
| **Füllstandsmessung springt sporadisch auf 0 oder Max** | Ultraschallstrahl trifft auf Tankinnenwand oder Einlaufrohr | Sensor zentrisch im Tankdeckel montieren; Beruhigungsrohr (PVC-Rohr mit Ausgleichsbohrungen) installieren |
| **Pumpe / Ventil schaltet trotz 100% PWM nicht ein** | Logiklevel des MOSFET reicht bei 3.3V nicht aus | Echten Logic-Level FET (IRLZ44N) oder Gate-Treiber-Transistor (z.B. BC547) vorschalten |
| **ESP32 stürzt bei Ventil-Abschaltung unregelmäßig ab (Brownout)** | Rückinduktion oder Spannungseinbruch auf 5V Schiene | Freilaufdiode prüfen; separaten 1000µF Low-ESR Elko an 12V Schiene und 100nF Keramikkondensator nahe am ESP32 setzen |
| **Dauerhaftes Überschwingen bei Sollwertsprung** | Integrator-Windup aktiv | Prüfen, ob Anti-Windup Clamping im C++ Sketch aktiviert ist |
| **Ständiges Regeln bei ruhendem Wasser** | ADC-Rauschen oder Wellen | Exponential Moving Average (EMA) Softwarefilter aktivieren ($y_{filt} = 0{,}85 \cdot y_{old} + 0{,}15 \cdot y_{raw}$) |

---

## 9. Anlagen-Spezifikation (Aktuelle Konfigurationsparameter)

Nachfolgende Parameter sind im aktuellen Parametersatz der Anlage konfiguriert:

### Regelparameter:
* **Proportionalbeiwert ($K_p$):** `3.500`
* **Integralbeiwert ($K_i$):** `0.1500\text{ s}^{-1}`
* **Differenzialbeiwert ($K_d$):** `0.8000\text{ s}`
* **Abtastzeit ($T_a$):** `100\text{ ms}`
* **Anti-Windup:** `AKTIV (Clamping)`
* **Derivative-on-Measurement:** `AKTIV (Schutz vor Kick)`
* **Totband / Hysterese:** `±0.30\text{ cm}`

### Behältergeometrie & Hydraulik:
* **Gesamthöhe ($h_{max}$):** `100.0\text{ cm}`
* **Behälterdurchmesser ($D$):** `50.0\text{ cm}`
* **Behältervolumen:** `196.3\text{ Liter}`
* **Aktueller Sollwert:** `60.0\text{ cm}` (`117.8\text{ Liter}`)
* **Maximaler Pumpen-/Ventilzulauf:** `2.50\text{ L/s}`
* **Maximaler Schwerkraftablauf:** `1.20\text{ L/s}`
* **Ablaufventilöffnung (Störgröße):** `35\%`
* **Ventilzeitkonstante ($T_{valve}$):** `0.30\text{ s}`

### Elektronik & ESP32:
* **Sensor-Typ:** `JSN-SR04T / HC-SR04 (Wasserdichter Ultraschallsensor)`
* **Aktor-Typ:** `Proportionalventil / DC-Pumpe via PWM (LEDC Peripherie)`
* **Aktor-PWM Pin:** `GPIO 25`
* **PWM-Frequenz:** `5000\text{ Hz}`
* **PWM-Auflösung:** `10\text{ Bit}`
* **FreeRTOS Tasking:** `AKTIVIERT (Core 1, vTaskDelayUntil)`

---

## 10. Anhang: Schnelle Checkliste für die Werkbank

- [ ] **Masseverbindung:** GND des ESP32 mit GND des 12V/24V Netzteils verbunden.
- [ ] **Pegelschutz:** 5V Echo-Ausgang des Ultraschall-Moduls über Spannungsteiler (oder Logic-Level-Shifter) auf maximal 3.3V begrenzt.
- [ ] **Freilaufdiode:** Diode antiparallel (Kathode an Plus, Anode an Drain) über der Ventilspule/Pumpe installiert.
- [ ] **Trockenlauf:** Pumpe bzw. Ventil vor Testlauf entlüftet und mechanisch freigängig.
- [ ] **Not-Aus:** Manueller Hauptschalter für die Ventil-Stromversorgung griffbereit.

---
*Dokumentation automatisch generiert durch das ESP32 Tank-PID Studio.*
