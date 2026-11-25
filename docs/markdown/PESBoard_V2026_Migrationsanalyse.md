# PES-Board V2026 – Migrationsanalyse und TODOs

**Stand:** 25. November 2026
**Basis:** Statische Analyse des Branches `dev_abuj_PESBoard2026`, Git-Archiv `6c91e274f6ca86bbd7f9b0f7e848aa4b0a388ac0`; ergänzt um die nachfolgenden lokalen Änderungen und Testergebnisse vom 25.11.2026 (neuer Commit-Hash noch nicht dokumentiert).
**Ziel:** Migration der Bibliotheken, Lösungsbeispiele und Lehrunterlagen auf V2026 unter Erhalt der V2019/V2023-Kompatibilität.

Dieses Dokument übernimmt die Befunde der vorangegangenen statischen Migrationsanalyse. Die Pinmap und die Liste der 24 Beispiele wurden für dieses Dokument nochmals mit dem Archiv abgeglichen. Die statischen Befunde zu den 24 Beispielen bleiben bestehen. Der nachfolgende gemeinsame Hardwaretest wurde auf V2026 erfolgreich durchgeführt; die älteren Versionen wurden nur kompiliert. Die Aufwandsschätzung ist eine Planungsschätzung, kein gemessener Aufwand.

## 1. Ausgangslage

| Bereich | Stand |
|---|---|
| Pin-Mapping V2026 | Implementiert |
| DC-Motoren PWM/DIR, Encoder, drei parallele Regler | Hardwaretest bestanden |
| Servo-PWM | Hardwaretest bestanden |
| Stepper-Timer | Hardwaretest bestanden (Pulserzeugung, nicht vollständiger Treibertest) |
| Stepper mit Motion Planner | V2026: beide Motoren gleichzeitig, Rampen, Richtungswechsel, interne Zielschritte und visuelle Bewegung PASS; V2025 offen |
| SPI1/SPI2 | Loopback bestanden |
| FTDI/USART3 | Loopback bestanden |
| ADC | Hardwaretest bestanden |
| LSM6DSOX über SPI | V2026: WHO_AM_I 0x6C, Initialisierung und 300 plausible Samples ohne Testfehler PASS; Achsen/Kalibrierung/Filter offen |
| Gemeinsames Hardwaretestprogramm | Alle drei Build-Konfigurationen: bereinigter Build PASS; V2026: Hardwaretest vor Entfernung des Debug-Printf PASS |
| SD-Capability V2025 | Korrigiert (`PB_HAS_SDCARD=1`) |
| Default-Versionskommentar | Korrigiert: V2019/V2023 |
| V2019/V2023-Lösungsbeispiele | 24/24 Builds erfolgreich laut bisherigem Testverlauf |
| Standardprogramm | Original aus `docs/solutions/main_base.cpp` wiederhergestellt; `platformio.ini` zurückgesetzt; abschliessender Default-Build PASS |
| V2026-Lösungsbeispiele | Vollständige Migration und Regression offen |
| Lehrunterlagen | Migration offen |

Ein erfolgreicher Build ist kein Nachweis der funktionalen Kompatibilität. Pin-Konflikte, Signalpolaritäten, Timerbelegung und Schnittstellen müssen separat geprüft werden. V2019/V2023 bleiben ohne Versionsflag die Default-Konfiguration. V2025 separat betrachten; keine vollständige V2025-Freigabe aus dieser Analyse ableiten.

### 1.1 Gemeinsamer Hardwaretest vom 25.11.2026

Testprogramm: `docs/solutions/test scripts/main_test_all_versions.cpp`. Testanleitung und detaillierte Ergebnisse: `docs/markdown/PESBoard_Hardwaretest.md`.

| Board | Build | Hardwaretest |
|---|---|---|
| V2019 | PASS (gemeinsamer Build mit V2023) | Verschoben |
| V2023 | PASS (gemeinsamer Build mit V2019) | Verschoben |
| V2025 | PASS | Verschoben |
| V2026 | PASS (bereinigte Fassung) | PASS im beschriebenen Umfang |

- [x] Drei DC-Motoren, Encoder, Drehzahl-/Positionsregelung und Motion Planner gleichzeitig geprüft (10/10 gültige Endphasen-Samples pro Motor und Phase).
- [x] Vier Servos visuell geprüft.
- [x] Zwei Stepper mit Motion Planner: 3200 Schritte/U, 5 U/s, 5 U/s²; Ziele ±19'200 und 0 intern exakt erreicht, mechanische Bewegung visuell bestätigt.
- [x] SD-SPI-Loopback: 30'000 Transfers, 0 Fehler.
- [x] LSM6DSOX: 300 Samples, 0 Fehler gemäss Testkriterien.
- [x] A0–A3 floating protokolliert (300 Samples/Kanal); keine Genauigkeitsprüfung.
- [x] DC-Debug-Printf entfernt und Testprogramm dauerhaft gesichert.
- [x] Bereinigte Fassung für V2026 erfolgreich kompiliert; kein erneuter Hardwarelauf nach Entfernung des Debug-Printf durchgeführt.
- [x] Standardprogramm und Standardkonfiguration wiederhergestellt; abschliessender Default-Build PASS.
- [ ] V2019, V2023 und V2025 auf physischer Hardware testen (verschoben).

Die Stepper-Zähler sind keine mechanische Rückmeldung; Servo-Pulsbreiten wurden im finalen Lauf nicht separat nachgemessen. Der SD-Loopback ersetzt keinen Test mit einer realen Karte. Der frühere FTDI-Loopback ist bestanden, war aber nicht Bestandteil dieses neuen kombinierten Laufs.

## 2. Pin-Konflikte und Anschlusskonzept

| Pin | Alte Verwendung in Beispielen bzw. Alias | V2026-Funktion |
|---|---|---|
| PB_9 | Status-LED, I2C SDA | Stepper 1 STEP (PB_9_ALT1) |
| PB_8 | RGB-LED, I2C SCL | Stepper 2 STEP (PB_8_ALT1) |
| PB_10 | UART TX, RGB-LED | IMU SPI SCK |
| PC_5 | Externer Taster, UART RX | Stepper Enable |
| PB_3 | Farbsensor-Frequenzeingang | SD SPI SCK |
| PB_14 | Farbsensor-LED | IMU SPI MISO |
| PA_4 | Farbsensor S0 | IMU CS |
| PB_4 | Alter Stepper STEP | SD SPI MISO |
| PA_7 | Alter Stepper DIR | Encoder M1 B |
| PC_10 | V2026-Status-LED-Alias | FTDI UART TX / alternatives SPI3 |
| PC_13 | V2026-Mechaniktaster-Alias | Nucleo User Button |

- [ ] V2026-Schaltplan und tatsächlich zugängliche Anschlüsse prüfen.
- [ ] Konfliktfreies Anschlusskonzept für LEDs, externe Taster, I2C, Farbsensor und RGB-LED definieren.
- [ ] Status-LED-Alias PC10 und Mechaniktaster-Alias PC13 auf Doppelbelegung prüfen; keine neuen Pins ohne Schaltplan festlegen.
- [ ] CAN/DC-Motor-Auswahl über PA11/PA12 und alternatives SPI3/USART3 berücksichtigen.
- [x] Kopfkommentar in `include/PESBoardPinMap.h` korrigiert: Default ist V2019/V2023, nicht V2026.
- [x] SD-Capability für V2025 auf `PB_HAS_SDCARD=1` korrigiert.
- [ ] Hardwarefähigkeiten und Versionsflags konsistent verwenden.

## 3. Lösungsbeispiele – vollständige Arbeitsliste

Alle 24 Beispiele sind betroffen oder benötigen eine gezielte Prüfung. Die Tabelle beschreibt den Migrationsbedarf; einzelne als Prüfung markierte Punkte können sich als bereits korrekt erweisen.

| Datei unter `docs/solutions/` | TODO |
|---|---|
| `main_base.cpp` | Externe Status-LED; anschliessend `src/main.cpp` synchronisieren |
| `main_color_sensor.cpp` | Sensor-Pin-Mapping, Status-LED |
| `main_color_sensor_with_rgb_led.cpp` | Sensor- und RGB-Pin-Mapping |
| `main_comp_filter.cpp` | SPI-IMU, UART, Status-LED |
| `main_dc_motor.cpp` | DIR-Pin, PWM-Semantik, Status-LED |
| `main_dd_kinematic_calib.cpp` | DIR-Pins, Status-LED |
| `main_gimbal.cpp` | SPI-IMU |
| `main_gimbal_1d_mahony.cpp` | SPI-IMU |
| `main_gimbal_3d_mahony.cpp` | SPI-IMU, Orientierung und Magnetometer prüfen |
| `main_ir_sensor.cpp` | Analogeingang, Status-LED |
| `main_ir_sensor_class.cpp` | Analogeingang, Status-LED |
| `main_line_follower.cpp` | DIR-Pins, I2C-Anbindung |
| `main_line_follower_base.cpp` | DIR-Pins, I2C-Anbindung |
| `main_my_dc_motor.cpp` | Motor-Treiber, DIR-Pin, Status-LED |
| `main_pes_monster.cpp` | Status-LEDs, Taster, IMU und optionale Peripherien; vorhandene DIR-Unterstützung prüfen |
| `main_sd_card_logger.cpp` | Status-LED, SD-Funktionstest |
| `main_sd_card_logger_with_time.cpp` | Status-LED, Analogeingang, SD-Funktionstest |
| `main_serial_stream.cpp` | FTDI-Pins, Status-LED |
| `main_servo.cpp` | Status-LED, Servo-Funktionstest |
| `main_stepper_motor.cpp` | STEP/DIR, Enable, Reset |
| `main_ws2_p2.cpp` | Externer Taster, Status-LED |
| `main_ws2_p2_ir_sensor.cpp` | Externer Taster, Status-LED |
| `main_ws3_p2.cpp` | DIR-Pin, externer Taster, Status-LED |
| `main_ws3_p2_ir_sensor.cpp` | DIR-Pin, externer Taster, Status-LED |

### 3.1 DC-Motoren

- [ ] Alte DCMotor-Konstruktoren versionsabhängig um DIR ergänzen (`#if PB_DC_CONTROL_PWM_DIR`, nicht `#ifdef` für das numerische Capability-Makro).
- [ ] Direkte PWM-Ansteuerung und auskommentierte Lehrbeispiele korrigieren.
- [ ] `Motor` und `MyDCMotor` für PWM/DIR erweitern, ohne bipolare PWM zu entfernen.
- [ ] Enable-Polarität, sichere Initialisierung, Drehrichtung und Fault-Verhalten überprüfen.

```cpp
// Alt: bipolar
DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1,
                 gear_ratio, kn, voltage_max);
// V2026: PWM/DIR
DCMotor motor_M1(PB_PWM_M1, PB_DIR_M1,
                 PB_ENC_A_M1, PB_ENC_B_M1,
                 gear_ratio, kn, voltage_max);
```

| Eigenschaft | V2019/V2023 | V2026 |
|---|---|---|
| Stillstand | 50 % PWM | 0 % PWM |
| Richtung | PWM-Tastgrad | DIR |
| Spannungsbetrag | Abstand zu 50 % | PWM-Tastgrad |
| Fault | Kein definierter Pin | PB_DC_MOT_STATE |

V2026 besitzt laut Pinmap keine DC-Motor-Strommessung. Alte Feedback-Annahmen dürfen nicht übernommen werden.

#### Separates TODO: DC-Regleroptimierung

- [x] V2026-Hardwaretest mit 80 % Integratorbegrenzung für alle drei DC-Regler bestanden; Änderung nur im Testprogramm. Mit der ursprünglichen Begrenzung liefen M1/M2 nicht an, M3 blieb mit etwa 4° Positionsabweichung stehen, als nur M1/M2 auf 60 % erhöht waren.
- [ ] Reststellgrössen bei Stillstand untersuchen: im finalen Lauf bei Sollgeschwindigkeit 0 ca. −2.014 V (M1), −4.632 V (M2), +3.629 V (M3). Das sind berechnete Reglerausgänge, keine gemessenen Motorspannungen.
- [ ] Tatsächliche Motorspannung, Motorstrom und Erwärmung extern messen; Integratorverhalten, Stillstandsstrategie und Reglertuning separat validieren. Keine pauschale Übernahme der 80 % in die Bibliothek.

### 3.2 IMU

- [ ] I2C-Konstruktoren versionsabhängig durch SPI-Initialisierung ergänzen (`PB_HAS_IMU_SPI`).
- [ ] Lebensdauer von SPI und Chip-Select sicherstellen.
- [x] LSM6DSOX real testen: SPI-Kommunikation, ID und grobe Messwertplausibilität im kombinierten V2026-Test bestanden.
- [ ] Achsenorientierung, Vorzeichen, Kalibrierung und Fehlerbehandlung separat validieren.
- [ ] Complementary-/Mahony-Filter und Gimbal-Beispiele überprüfen; fehlendes Magnetometer berücksichtigen (insbesondere absolute Gierorientierung).

```cpp
// Alt
IMU imu(PB_IMU_SDA, PB_IMU_SCL);
// V2026 – API entsprechend dem vorhandenen Treiber prüfen
SPI imu_spi(PB_IMU_MOSI, PB_IMU_MISO, PB_IMU_SCK);
DigitalOut imu_cs(PB_IMU_CS, 1);
IMU imu(imu_spi, imu_cs);
```

### 3.3 Stepper

Altes Beispiel: `Stepper stepper_M1(PB_9, PB_8, 400 * 16);` und `Stepper stepper_M2(PB_4, PA_7, 400 * 16);`. Die neue Klasse erwartet die TIM10/TIM11-Pinvarianten; alte Zuweisungen können trotz Build zur Assertion führen.

```cpp
Stepper stepper_M1(PB_STEP_STEPPERMOTOR1, PB_DIR_STEPPERMOTOR1, 400 * 16);
Stepper stepper_M2(PB_STEP_STEPPERMOTOR2, PB_DIR_STEPPERMOTOR2, 400 * 16);
```

- [ ] Zentrale Pinmap statt fester Pins verwenden.
- [ ] Gemeinsames Enable (PC5) und Reset (PA5), beide aktiv-low, sicher initialisieren.
- [x] V2026: Motion Planner, Rampen, Richtung, Stoppen und beide Motoren gleichzeitig im gemeinsamen Hardwaretest geprüft.
- [ ] Migriertes Lösungsbeispiel separat prüfen; Schrittauflösung an tatsächlich verwendeten Motor/Treiber anpassen (Testprogramm: 3200 Schritte/U).
- [ ] V2025: Stepper-Bibliothek um geeignete Impulserzeugung mit Motion Planner und Positionsprüfung erweitern; aktuelles Testprogramm verwendet nur GPIO/Ticker mit 50/80 Hz. Hardwaretest und Erweiterung verschoben.
- [ ] Unterstützungsumfang älterer Stepper-Anwendungen klären: V2019/V2023 markiert die Pinmap als ohne Stepper-Unterstützung; V2025 verwendet andere Pins/Timer. Keine nicht unterstützte Kombination stillschweigend freigeben.

### 3.4 Farbsensor

| Signal | Bisher | V2026-Funktion |
|---|---|---|
| LED | PB14 | IMU MISO |
| S0 | PA4 | IMU CS |
| S1 | PB0 | A3 |
| S2 | PC0 | A1 |
| S3 | PC1 | A0 |
| Frequenzausgang | PB3 | SD SCK |

- [ ] Fest codierte Default-Pins in `lib/ColorSensor/ColorSensor.h` behandeln.
- [ ] Vorhandenen Konstruktor mit sechs wählbaren Pins nutzen; Sensorlogik möglichst unverändert lassen.
- [ ] Anschlusskonzept mit zusätzlicher RGB-LED klären und Frequenzmessung testen.

### 3.5 Line-Follower

- [ ] I2C auf PB9/PB8 ersetzen; dort liegen V2026-Stepper-Signale.
- [ ] Externes I2C-Anschlusskonzept anhand Schaltplan definieren; derzeit kein V2026-I2C-Alias vorhanden.
- [ ] Beide Line-Follower-Beispiele inklusive Motorsteuerung testen.

### 3.6 SerialStream und weitere Peripherien

- [ ] UART-Beispiele von PB10/PC5 auf V2026 FTDI PC10/PC11 migrieren; keine gleichzeitige Status-LED auf PC10.
- [ ] Analogeingänge über `PB_Ax` statt alte feste Pins zuweisen; PC2 ist auf V2026 A2.
- [ ] SD-Karte mit realem Medium testen (SPI-Loopback genügt nicht).
- [ ] Servo-Beispiele auf Hardware-PWM/Timer und gleichzeitige Nutzung prüfen.

## 4. Bibliotheken

| Bibliothek | Befund / TODO |
|---|---|
| `Motor` | Zwingend: bipolare PWM um PWM/DIR ergänzen; 50 % bei 0 V wäre auf V2026 falsch |
| `MyDCMotor` | Zwingend: Motor und DIR-Konstruktor migrieren |
| `ColorSensor` | Zwingend: alte Default-Pins behandeln |
| `Stepper` | V2026 Motion Planner und beide Motoren funktional getestet; V2025-Unterstützung und Beispielmigration offen |
| `IMU` | SPI vorhanden; Achsen, Fehlerbehandlung, Magnetometer prüfen |
| `DCMotor` | PWM/DIR vorhanden, Hardwaretest bestanden (80 % Integratorlimit im Test); Beispiele migrieren, Reststellgrössen und allgemeines Tuning separat prüfen |
| `EncoderCounter` | Neue Pins implementiert und getestet |
| `Servo` | Hardware-PWM V2026 vorhanden; vier Servos gleichzeitig visuell geprüft; Beispielregression offen |
| `LSM6DSOX` | SPI-Treiber vorhanden; reale ID-/Plausibilitätsprüfung bestanden, weitergehende Sensorvalidierung offen |
| `FastPWM` | Bereits im Motorentest eingesetzt; Regression |

Kein unmittelbarer hardwarebedingter Änderungsbedarf in der bisherigen Analyse erkannt: `AvgFilter`, `IIRFilter`, `MedianFilter3`, `LinearCharacteristics3`, `PIDCntrl`, `Motion`, `Mahony`, `Chirp`, `GPA`, `DebounceIn`, `Encoder`, `PwmIn`, `IRSensor`, `UltrasonicSensor`, `LineFollower`, `SensorBar`, `SDLogger`, `SDWriter`, `SerialPipe`, `SerialStream`, `RealTimeThread`, `ThreadFlag`, Eigen. Dies ist keine Funktionsfreigabe; abhängige Anwendungen bleiben zu testen.

## 5. Dokumentation und Lehrunterlagen

| Datei unter `docs/markdown/` | TODO |
|---|---|
| `main_description.md` | Status-LED |
| `dc_motor.md` | PWM/DIR, Treiberschaltung, Konstruktoren |
| `my_dc_motor.md` | Motor-Implementierung, UART |
| `imu.md` | LSM6DSOX, SPI, Achsen, Magnetometer |
| `1d_mahony.md` | Sensorannahmen prüfen |
| `stepper_motor.md` | STEP/DIR, Enable, Reset, Motion Planner |
| `serial_stream.md` | FTDI-Anschluss |
| `sd_card_logger.md` | SPI-Pinbelegung |
| `line_follower.md` | I2C-Anschluss |
| `color_sensor.md` | Anschlusskonzept |
| `ir_sensor.md` | Analogeingänge |
| `servo.md` | Hardware-PWM, Timer |
| `dd_kinematics.md` | Motorkonstruktoren prüfen |
| `ultrasonic_sensor.md` | Anschlussdarstellung prüfen |
| `ws1.md` | LED-Anschluss |
| `ws2.md` | Taster, LED |
| `ws3.md` | Taster, LED, Motoransteuerung |
| `ws4.md`, `ws5.md` | Abhängigkeiten zu migrierten Beispielen |
| `changes_for_flipped_classroom.md` | Aufgaben, Codebeispiele, Anschlussangaben |

- [ ] `README.md`: V2026-Peripherieübersicht ergänzen.
- [ ] Alte Pinouts und Schaltpläne versionsabhängig kennzeichnen.
- [ ] V2026-Schaltplan, Pinout, relevante Datenblätter ergänzen.
- [ ] MPU6500-Testprogramm unter `docs/solutions/test scripts/` versionsspezifisch kennzeichnen (bisheriger Befund: V2025).
- [ ] Lehrtexte und Abbildungen mit finalen Beispielen abgleichen.

## 6. Reihenfolge und aktualisierte Aufwandsschätzung

Personenstunden für eine mit dem Projekt vertraute Entwicklungsperson, mit KI-Unterstützung, vorhandener Hardware und funktionierender Toolchain. Bereits erledigte Hardware- und Legacy-Build-Tests werden nicht nochmals als Entwicklungsaufwand angesetzt; gezielte Regression ist enthalten. Die Arbeitspakete sind abgegrenzt, um Doppelzählung zu vermeiden. Ursprüngliche Schätzung: 34 / 67 / 132 h (optimistisch / realistisch / erschwert). Nach der bestandenen V2026-Validierung werden die Pakete Stepper, IMU und integrierte Regression moderat reduziert; die eigentliche Migration der 24 Beispiele und Lehrunterlagen bleibt offen. Die folgenden Werte sind eine neue Planungsschätzung, keine gemessenen Zeiteinsparungen.

| Nr. | Arbeitspaket | Optimistisch | Realistisch | Erschwert |
|---|---|---:|---:|---:|
| 1 | Pinmap, Schaltplan, Anschlusskonzept | 2 | 4 | 8 |
| 2 | Standardprogramm und Basis-LED/Taster-Konzept | 1 | 2 | 4 |
| 3 | DC-Motor-Bibliotheken und Beispiele | 4 | 7 | 12 |
| 4 | Stepper-Beispielmigration und gezielte Regression V2026 | 2 | 4 | 9 |
| 5 | IMU-Beispiele, weitergehende Sensorvalidierung, Filter und Gimbal | 3 | 6 | 13 |
| 6 | Weitere Peripherien (IR, Servo, SD, Farbe, I2C, UART) | 5 | 10 | 20 |
| 7 | Workshops und PES Monster (Code) | 3 | 6 | 12 |
| 8 | Build-Matrix und Regression | 2 | 4 | 8 |
| 9 | Integrierte Hardware-Regression der migrierten Anwendungen und Fehlerkorrekturen | 3 | 6 | 12 |
| 10 | Lehrdokumentation, Pinouts, Abbildungen, README | 6 | 12 | 24 |
| | **Gesamt (h)** | **31** | **61** | **122** |

- Optimistisch: ca. 3,7 Arbeitstage bei 8,4 h/Tag.
- Realistisch: ca. 7,3 Arbeitstage bei 8,4 h/Tag.
- Erschwert: ca. 14,5 Arbeitstage bei 8,4 h/Tag.
- Empfohlenes Planungsbudget: rund **75 Stunden (ca. 8,9 Arbeitstage)**, inklusive rund 23 % Reserve.

Grösste Unsicherheiten: physische Verfügbarkeit konfliktfreier externer Pins, weitergehende IMU-/Filtervalidierung, Stepper-Kompatibilität älterer Hardware, Umfang der Lehrgrafiken und Fehler aus kombinierten Hardwaretests. Falls Adapter, PCB-Änderungen, Beschaffung oder eine umfassende V2025-Freigabe nötig werden, ist deren zusätzlicher Aufwand nicht enthalten. Separat zu schätzen und nicht im obigen V2026-Migrationsbudget enthalten: verschobene vollständige Hardwaretests V2019/V2023/V2025, V2025-Stepper-Erweiterung und allgemeine DC-Regleroptimierung. Ebenso nicht enthalten: grundlegende Neuentwicklung der Lehrveranstaltung oder allgemeines Refactoring ohne Migrationsbezug.

## 7. Abnahmekriterien

- [ ] Verbindliches Anschlusskonzept und korrigierte Pinmap vorhanden.
- [ ] Default-Build weiterhin V2019/V2023; alle 24 Legacy-Beispiele erneut erfolgreich kompilieren.
- [ ] Alle für V2026 vorgesehenen Beispiele kompilieren mit `PESBOARD_V2026`; nicht unterstützte Kombinationen explizit dokumentieren.
- [ ] Keine unbeabsichtigten Pin-/Timer-Konflikte und keine falsche PWM-/Enable-Semantik.
- [ ] Reale Funktionstests für Motoren, Stepper, IMU, Servo, SD, UART, Farbsensor, Line-Follower und Workshops entsprechend ihrer tatsächlichen Hardwareanforderungen.
- [ ] `src/main.cpp` und `docs/solutions/main_base.cpp` konsistent.
- [ ] Dokumentation und Lehrmaterial stimmen mit Code und Hardware überein.
- [ ] Änderungen in nachvollziehbaren, thematischen Commits; keine automatischen Pushes.

## 8. Nächster Schritt / Blocker

Vor Abschluss des aktuellen Test-Arbeitspakets: Git-Diff kontrollieren und thematischen Commit erstellen. Bereinigter V2026-Testbuild und abschliessender Default-Build sind bestanden. Kein automatischer Push.

Für die anschliessende Migration: V2026-Schaltplan bereitstellen und zunächst die freien bzw. extern zugänglichen Pins verbindlich klären. Ohne diese Grundlage keine willkürliche Neuzuordnung von Status-LED, Taster, I2C oder Farbsensor. Danach Arbeitspakete einzeln umsetzen und prüfen. Diese Aktualisierung dokumentiert den erreichten Stand; keine weiteren Änderungen an Firmware oder Lehrbeispielen durch diese Analyse.
