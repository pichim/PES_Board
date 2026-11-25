# PES-Board: gemeinsamer Hardwaretest V2019/V2023/V2025/V2026

**Stand: 25.11.2026.** Gemeinsames Testprogramm mit erfolgreich kompilierten Konfigurationen für V2019/V2023, V2025 und V2026. Der kombinierte Hardwaretest V2026 wurde inklusive visueller Servo- und Stepper-Prüfung bestanden. Hardwaretests der älteren Versionen sind verschoben.

| Boardversion | Build | Hardwaretest |
|---|---|---|
| V2019 | PASS (gemeinsamer Build) | Offen |
| V2023 | PASS (gemeinsamer Build) | Offen |
| V2025 | PASS | Offen |
| V2026 | PASS (bereinigte Fassung) | PASS im beschriebenen Testumfang |

Nach dem erfolgreichen V2026-Hardwaretest wurde der zusätzliche DC-Regler-Debug-Printf entfernt. Die bereinigte Fassung wurde anschliessend für alle drei Build-Konfigurationen erfolgreich kompiliert. Auch der abschliessende Default-Build des wiederhergestellten Standardprogramms war erfolgreich. Ein erneuter Hardwaretest nach Entfernung der Debug-Ausgabe wurde nicht durchgeführt.

## Installation

Empfohlener dauerhafter Pfad: `docs/solutions/test scripts/main_test_all_versions.cpp`. Zum Testen bei sauberem Git-Arbeitsstand vorübergehend nach `src/main.cpp` kopieren. Standardprogramm anschliessend aus `docs/solutions/main_base.cpp` wiederherstellen. Für den Test sind keine weiteren Änderungen an Treiberdateien oder Pinmap erforderlich. Vorher die lokale Build-Konfiguration sichern und nach dem Test wiederherstellen.

```powershell
Copy-Item 'docs/solutions/test scripts/main_test_all_versions.cpp' src/main.cpp -Force
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run
```

Versionsauswahl in `platformio.ini` unter den bestehenden `build_flags`, jeweils genau eine Definition:

| Hardware | Flag |
|---|---|
| V2019 / V2023 | `-DPESBOARD_V2019_2023` (oder kein Flag) |
| V2025 | `-DPESBOARD_V2025` |
| V2026 | `-DPESBOARD_V2026` |

Bei Wechsel der Version Clean-Build (`pio run -t clean`), anschliessend `pio run`. Die Definition muss für sämtliche Übersetzungseinheiten gelten. Keinesfalls eine Firmware mit falscher Boardversion aufstecken/ausführen. Beide Altboards separat physisch testen, obwohl sie denselben Build verwenden.

## Vorbereitung

- Nucleo F446RE, ST-LINK-VCP 115200 Baud. 12-V-Motorversorgung mit geeigneter Strombegrenzung und schnell erreichbarer Abschaltung. Mechanisch entkoppelte Motoren, freie Servohebel, keine Last oder Anschläge. Versorgung, Pinbelegung und Enable-Polarität vor dem ersten Lauf je Version verifizieren. Die Software ist kein unabhängiger Not-Aus.
- Testmotoren M1 156:1, M2 31:1, M3 78.125:1, 20 Encodercounts/Motorumdrehung, 12 V, KN=15 rpm/V wie im bisherigen Test. Andere Motoren erfordern angepasste Parameter.
- Vier Servos D0–D3 anschliessen oder mit Oszilloskop messen. Auf V2025 teilt D3 PC13 mit dem Nucleo-Userbutton: elektrische Verträglichkeit vor Anschluss prüfen, Taste nicht betätigen. Kein Userbutton als Startsignal.
- A0–A3 offen lassen; keine Stepper-Loopbacks an ADC-Eingänge anschliessen.
- SD-Karte und andere Teilnehmer am zu testenden SPI-Bus entfernen. Nur MOSI mit MISO verbinden (bei ausgeschalteter Hardware); CS bleibt High. Keine Verbindungen zu Versorgung oder SCK herstellen. Physische Zugänglichkeit vorab prüfen.
- V2026: IMU angeschlossen lassen; auf IMU-SPI **keinen** Loopback setzen. PA11/PA12 müssen hardwareseitig als Motor-PWM/Fault geroutet sein (nicht CAN).
- V2025: PH1-Fault-Eingang standardmässig nicht konfiguriert. Optional `-DPES_TEST_V25_FAULT_ACTIVE_LOW=1` nach Verifikation der Hardwarebeschaltung und aktiven Low-Polarität. Altboards haben laut Pinmap keinen entsprechenden Fault-Pin.

## SPI-/SD-Loopback

MCU-Pins laut Pinmap (physische Steckerbelegung am jeweiligen Board überprüfen):

| Hardware | MOSI → MISO | SCK | CS (High) |
|---|---|---|---|
| V2019/V2023 | PC12 → PC11 | PC10 | PD2 |
| V2025 | PB5 → PB4 | PB3 | PA10 |
| V2026 | PB5 → PB4 | PB3 | PB13 |

V2025 besitzt ebenfalls einen SD-Kartenanschluss. Die Pinmap wurde bereits korrigiert: `PB_HAS_SDCARD=1` für V2025. Der Test verwendet bereits die `PB_SD_*`-Pins für alle Boardversionen. Loopback testet die digitale Datenübertragung und nicht SD-Dateisystem, Kartenversorgung, CS-Funktion oder eine reale Karte.

## Ablauf

1. Start/Reset mit geeigneter Hardwarekonfiguration. Terminal 115200 öffnen.
2. `T` senden: Peripherien initialisieren, DC- und Stepper-Treiber bleiben deaktiviert.
3. `R` senden: drei Testphasen starten, je mindestens 3 s bei V2019/V2023/V2025 bzw. 5 s bei V2026 (Ausgabe und Kommunikation verlängern die Laufzeit).
4. Servo-/STEP-Signale während der Phasen messen; danach Summary auswerten. Alle Motorstufen werden bei regulärem Abschluss oder erkanntem Abbruch deaktiviert. Bei Hänger/ungewöhnlicher Bewegung sofort Versorgung abschalten.

| Phase | M1 (rev/s) | M2 (rev/s) | M3 (rel. Grad) | D0/D1/D2/D3 (µs) |
|---|---:|---:|---:|---|
| A | +0.20 | +0.50 | +45 | 1250/1400/1600/1750 |
| B | -0.20 | -0.50 | -45 | 1750/1600/1400/1250 |
| C | 0 | 0 | 0 | 1500/1500/1500/1500 |

DC-PWM 20 kHz, M1 ohne Motion Planner, M2/M3 mit Motion Planner. Nur für V2026 wird die Integratorbegrenzung aller drei DC-Regler im Testprogramm auf 80 % gesetzt. Dies ist keine allgemeine Änderung oder Freigabe des Reglertunings. Automatische Bewertung: mindestens 8 der letzten 10 Messwerte je Phase innerhalb ±0.05 rev/s (M1), ±0.10 rev/s (M2), ±5° (M3); zusätzlich signierter Encoderweg bei M1/M2. Abbruch bei unplausibler Rückmeldung, unzureichender Bewegung oder aktivem konfiguriertem Fault.

SPI: 4 MHz, Mode 0, 10 Bitmuster je 10 Wiederholungen pro Sample, bei vollständigem Lauf gelten die versionsabhängigen Mengen in der folgenden Tabelle; null Fehler erforderlich.

| Messgrösse | V2019/V2023/V2025 | V2026 |
|---|---:|---:|
| Samples je Phase | 60 | 100 |
| Mindestdauer je Phase | 3 s | 5 s |
| SPI-Transfers insgesamt | 18'000 | 30'000 |
| IMU-Samples insgesamt | 180 | 300 |
| ADC-Samples pro Kanal | 180 | 300 |

IMU: physischer Sensor über den versionsspezifischen Low-Level-Treiber, Identifikation und Messwerte während des gleichzeitigen Betriebs. V2019/V2023: LSM9DS1 (I²C, inklusive Magnetometer); V2025: MPU6500 (I²C); V2026: LSM6DSOX (SPI). Beschleunigung in m/s², Gyro in rad/s. Grobe Plausibilitätsprüfung (1–35 m/s² Beschleunigungsnorm, Gyronorm unter 40 rad/s, bei LSM9DS1 zusätzlich nichttriviale Magnetometernorm). Sensor bei laufenden Motoren nicht von Hand bewegen. Achsenorientierung, Bewegungserkennung, Kalibrierung, Mahony-Filter und Aktualität der Daten bei Kommunikationsfehlern sind separat zu prüfen; ältere Treiber liefern keine verlässliche Fehleranzeige pro Transfer. Kein vollständiger IMU-Freigabetest.

ADC: 180 bzw. 300 Samples pro Kanal gemäss obiger Tabelle bei vollständigem Lauf, Minimum/Maximum und Livewerte bei Floating. Kein elektrischer PASS/FAIL, keine Aussage über Genauigkeit oder Kanalzuordnung ohne definierte Eingangsspannung.

## Stepper

| Hardware | STEP1 / DIR1 | STEP2 / DIR2 | ENABLE / RESET |
|---|---|---|---|
| V2025 | PB12 / PB2 | PC6 / PC5 | PC8 / PB8 |
| V2026 | PB9_ALT1 / PB2 | PB8_ALT1 / PB15 | PC5 / PA5 |

### V2025: einfacher STEP-Signaltest

Zwei unabhängige Ticker/GPIOs erzeugen 50 Hz bzw. 80 Hz mit nominal 50 % Duty Cycle. Die Richtung wird in Phase B umgekehrt und in Phase C auf die ursprüngliche Richtung zurückgesetzt. Die Pulse laufen auch in Phase C; ein Positionstest findet nicht statt. Die Softwarezähler erfassen erzeugte GPIO-Flanken, keine unabhängige physische Rückmessung. Die aktuelle `Stepper`-Klasse unterstützt die V2025-STEP-Pins nicht; Motion Planner und Positionsprüfung bleiben offen.

### V2026: Stepper mit Motion Planner

Die `Stepper`-Klasse verwendet TIM11/TIM10 für die STEP-Impulserzeugung. Beide Motoren arbeiten mit Positionsvorgabe, Beschleunigungs- und Verzögerungsrampen.

- Konfigurierte Schrittauflösung: 3200 Schritte/Umdrehung (Treiber entsprechend einstellen).
- Maximalgeschwindigkeit: 5 U/s (16 kHz STEP bei dieser Auflösung).
- Beschleunigung und Verzögerung: 5 U/s².
- Nominelle STEP-High-Zeit der Bibliothek: 10 µs; kein fester 50-%-Duty-Cycle.

| Phase | STEP1 Ziel | STEP2 Ziel | Bewegung STEP1 / STEP2 |
|---|---:|---:|---|
| A | +19'200 | −19'200 | +6 / −6 U |
| B | −19'200 | +19'200 | −12 / +12 U |
| C | 0 | 0 | +6 / −6 U |

Die automatische Prüfung verlangt am Phasenende exakt die internen Zielschrittzahlen und Stillstand beider Stepper (`!isRunning()`). Interne Zähler sind keine mechanische Positionsrückmeldung; reale Bewegung, Richtungswechsel und Rückkehr zur Ausgangsposition sind zusätzlich visuell zu prüfen. Keine STEP-Loopbacks an ADC-Eingänge.

### Endstufen aktivieren (V2025/V2026)

Standardmässig bleiben die Stepper-Endstufen deaktiviert. Für den vollständigen mechanischen Test unter `build_flags` zusätzlich setzen:

```ini
  -DPES_TEST_ENABLE_STEPPER_DRIVERS=1
```

Nur bei verifizierter Enable-/Reset-Polarität, korrekt angeschlossenen unbelasteten Motoren, passender Schrittauflösung und geeigneter Strombegrenzung aktivieren. Der erfolgreiche V2026-Hardwaretest wurde mit diesem Flag durchgeführt. Ohne Flag kann die automatische interne Schrittprüfung bei V2026 auch ohne physische Motorbewegung bestehen.

## Abnahme pro physischem Board

- [ ] Korrekter Build mit richtiger Versionsdefinition.
- [ ] M1/M2/M3 und Encoder: PASS.
- [ ] D0–D3: alle vier PWM-Signale 50 Hz und Sollpulsbreiten (Messung), alternativ Servobewegung als qualitative Prüfung dokumentieren.
- [ ] IMU: ID und Plausibilität PASS; separates Bewegen/Kippen bei abgeschalteten Motoren zur Achsenprüfung.
- [ ] SPI-Loopback: 18'000 Bytes (V2019/V2023/V2025) bzw. 30'000 Bytes (V2026), 0 Fehler.
- [ ] A0–A3: Floating-Messwerte erfasst.
- [ ] V2025: STEP1/STEP2 Frequenz/Duty und DIR-Wechsel gemessen; bei aktiviertem Treiber Bewegung geprüft.
- [ ] V2026: STEP motion/count PASS; bei aktiviertem Treiber beide Bewegungen, Richtungswechsel und mechanische Rückkehr zur Ausgangsposition visuell geprüft.
- [ ] Endstufen nach Test deaktiviert, keine STEP-/Servopulse.
- [ ] Keine Resets, Hänger oder erkennbaren gegenseitigen Beeinflussungen.

Automatische Summary umfasst DC, SPI, IMU-Plausibilität und vollständigen Ablauf. Bei V2026 umfasst die automatische Summary zusätzlich die internen Stepper-Zähl- und Stillstandsprüfungen. Servo-Funktion, mechanische Stepper-Bewegung und ADC-Bewertung bleiben manuell. FTDI/UART-Loopback ist nicht enthalten (nicht Teil des aktuell angeforderten Umfangs); ST-LINK dient ausschliesslich der Bedienung/Diagnose.

## Bekannte Grenzen

- V2019/V2023/V2025 wurden bisher nur kompiliert. Laufzeitverhalten, Timerbelegung, RAM-/Threadbedarf, V2025-Pinbeschaltung und Enable-Polaritäten müssen auf den jeweiligen physischen Boards geprüft werden.
- Für den Hardwaretest wurden keine Treiberbibliotheken geändert. Die SD-Capability für V2025 wurde in der Pinmap korrigiert. Die 80-%-Integratorbegrenzung gilt nur im V2026-Testprogramm.
- Kein sicherheitsgerichteter Test; blockierende Treiberaufrufe können eine reguläre Abschaltung verzögern. Externe Abschaltung und geeignete Strombegrenzung sind erforderlich.

## Testergebnis V2026 – 25.11.2026

| Funktion | Ergebnis |
|---|---|
| M1 vorwärts / rückwärts | PASS: +0.201 / −0.195 U/s (letzte ausgegebene Werte je Phase) |
| M2 vorwärts / rückwärts | PASS: +0.481 / −0.485 U/s |
| M3 +45° / −45° / 0° | PASS: +45.4° / −44.9° / +0.2° |
| Motor-Messwerte in Solltoleranz | 10/10 je Motor in allen drei Phasen |
| Vier Servos | PASS (visuelle Funktionsprüfung; Pulsbreiten nicht separat nachgemessen) |
| Zwei Stepper mit Motion Planner | PASS (interne Zielschritte in allen Phasen exakt erreicht; Bewegung visuell bestätigt) |
| SD-SPI-Loopback | PASS: 30'000 Bytes, 0 Fehler |
| LSM6DSOX | WHO_AM_I = 0x6C, Initialisierung PASS |
| IMU-Samples | PASS: 300 Samples, 0 Fehler nach Testkriterien |
| IMU-Normen | Beschleunigung 7.97–9.90 m/s², Gyro 0.008–0.095 rad/s |
| ADC A0 / A1 / A2 / A3 | Floating-Werte erfasst (300 Samples je Kanal; keine Genauigkeitsprüfung) |
| Gleichzeitiger Betrieb | Automatischer Teiltest PASS; visuelle Servo-/Stepper-Funktionsprüfungen bestätigt |

### Offene TODOs

- [ ] V2019, V2023 und V2025 jeweils auf physischer Hardware testen.
- [ ] V2025-Stepper-Treiber um Motion Planner und Positionsprüfung erweitern, ohne V2026 zu beeinträchtigen.
- [ ] DC-Regleroptimierung separat untersuchen: Bei Sollgeschwindigkeit null verbleiben im V2026-Test beispielsweise −2.014 V (M1), −4.632 V (M2) und +3.629 V (M3) als berechnete Reglerausgänge. Tatsächliche Motorspannung, Strom, Erwärmung und Integratorverhalten bei Stillstand prüfen.
- [ ] Allgemeines Reglertuning und Integratorbegrenzung unabhängig vom Hardwaretest validieren.
- [ ] IMU-Achsenorientierung, Kalibrierung und Filterverhalten separat validieren.
- [ ] SD-Kartenfunktion mit realer Karte und Dateisystem separat testen.

Die V2026-Freigabe bezieht sich ausschliesslich auf den beschriebenen Hardwaretest, nicht auf alle Lösungsbeispiele oder sämtliche elektrischen Spezifikationen.
