# Changes for Flipped Classroom (WS1–WS3)

This document describes how WS1–WS3 are adapted to a flipped-classroom format:
- **Before class:** students build the minimum conceptual/tooling foundation.
- **In class:** time is reserved for hands-on building, debugging, calibration, and instructor/TA feedback.
- **Accountability:** short pre-class quiz to surface gaps early.

---

## QR code generator

[28/01 15:11] Sydow Antje (sydo)
https://www.qrcode-generator.ch/


---

## Source Mapping (from README)

Markdown files read and checked (with workshop tags and priorities):
- README.md (ws1-3, 1)
- build_mbed_linux.md
- build_mbed_windows.md
- stepper_motor.md
- course_setup.md (ws1, 2)
- dc_motor.md (ws3, 3)
- imu.md (ws5, 2)
- ir_sensor.md (ws1, 6)
- dd_kinematics.md (ws4, 2)
- line_follower.md (ws4, 3)
- main_description.md (ws1, 5)
- my_dc_motor.md
- sd_card_logger.md
- serial_stream.md (ws5, 3)
- servo.md (ws2, 3)
- tips.md (ws1, 3)
- ultrasonic_sensor.md (ws2, 4)
- ws1.md (ws1, 4)
- ws2.md (ws2, 2)
- ws3.md (ws3, 2)
- ws4.md (ws4, 1)
- ws5.md (ws5, 1)
- ws6.md (does not exist yet)

Ordered by workshop (same tags/priorities):
- WS1: README.md; course_setup.md (ws1, 2); tips.md (ws1, 3); ws1.md (ws1, 4); main_description.md (ws1, 5); ir_sensor.md (ws1, 6)
- WS2: README.md; ws2.md (ws2, 2); servo.md (ws2, 3); ultrasonic_sensor.md (ws2, 4)
- WS3: README.md; ws3.md (ws3, 2); dc_motor.md (ws3, 3)
- WS4: ws4.md (ws4, 1); dd_kinematics.md (ws4, 2); line_follower.md (ws4, 3)
- WS5: ws5.md (ws5, 1); imu.md (ws5, 2); serial_stream.md (ws5, 3)
- WS6: ws6.md (does not exist yet)
- Cross-workshop/ungrouped: build_mbed_linux.md; build_mbed_windows.md; stepper_motor.md; my_dc_motor.md; sd_card_logger.md

---

## Workshop ↔ File Mapping

- Overview
  - Reference index: [README.md](../../README.md)

---

## WS1

### Ordered reading (per README numbers)

1) [README.md](../../README.md)
2) [course_setup.md](course_setup.md)
3) [tips.md](tips.md)
4) [ws1.md](ws1.md)
5) [main_description.md](main_description.md)
6) [ir_sensor.md](ir_sensor.md)

Solutions:
- [../solutions/main_ir_sensor.cpp](../solutions/main_ir_sensor.cpp)
- [../solutions/main_ir_sensor_class.cpp](../solutions/main_ir_sensor_class.cpp)

---

### Pre-class (read + quiz + install check; no fork/clone/build)

**Target time:** ~110–155 min total (students can split into two sessions).

**Read (same order as mapping, with estimated read/understand time):**
- README.md (overview + safety) — ~5–10 min
- course_setup.md (tooling + import/build/flash flow) — ~15–20 min
- tips.md (programming hygiene + debugging) — ~10–15 min
- ws1.md (what happens in class) — ~8–12 min
- main_description.md (project scaffold + USER gating) — ~12–18 min
- ir_sensor.md (sensor theory + wiring + calibration overview) — ~25–35 min

**Quiz (MS Forms, ~7–10 min total):**
- 6–8 auto-graded items.
- Focus: wiring safety, AnalogIn scaling, sensor range awareness, calibration mapping, and one code-structure question.

**Install/check tools (~25–35 min):**
- Install Mbed Studio and open it once to confirm it starts.
- Verify accounts: GitHub + Mbed (MATLAB university login or Python/numpy/matplotlib for calibration).
- Fork/clone/build happens in class (Block 0).

**Pre-class outputs (what students should bring):**
- Quiz completed.
- Mbed Studio installed and opened once.
- Ready GitHub + Mbed accounts.
- Ready to identify IR sensor model + valid range (from `ir_sensor.md`).

---

### WS1 Quiz Outline (MS Forms)

- 6–8 items, auto-graded. Total quiz time ~7–10 min.

**Suggested questions (core 4)**
- Wiring safety (single choice): “Before connecting/reconnecting hardware, what must be true?”
  Options: a) Power off/disconnect USB; b) Use thicker wires; c) Switch to 5V servo rail; d) Enable motors.
  Answer: a.

- Analog pin / voltage range (single choice): “What voltage range does AnalogIn map to 0.0–1.0?”
  Options: a) 0–5 V; b) 0–3.3 V; c) 0–12 V; d) −3.3–3.3 V.
  Answer: b.

- Range awareness (single choice): “Why do we need to identify the sensor model and range first?”
  Options: a) To increase max range; b) To choose valid calibration points and avoid bad fits; c) To reduce CPU load; d) To change PWM frequency.
  Answer: b.

- Calibration purpose (single choice): “Why calibrate the IR sensor?”
  Options: a) To shift LED blink rate; b) To map sensor voltage to distance accurately; c) To increase max range beyond spec; d) To save flash memory.
  Answer: b.

---

### In-class (4 × 45 min)

**Block 0 (Setup):** fork repo, clone locally, import into Mbed Studio, select target NUCLEO-F446RE, build once.
- Output: build succeeds or students have a captured error screenshot/log.

**Block 1 (Part 1: LED + USER gating + code orientation):**
- Wire LED (PB_9 + resistor), flash, verify USER toggles behavior.
- Quick “code scavenger hunt”: find `do_execute_main_task`, find where `printf` is placed, locate the timing loop.

**Block 2 (Part 2: IR wiring + live signal + start data capture):**
- Identify IR sensor model and valid range.
- Wire IR sensor to PC_2, confirm mV printouts change with distance.
- Start filling distance/voltage table (dense near minimum).

**Block 3 (Calibration + implement mapping + mini demo):**
- Finish data capture (use a dense grid appropriate to the sensor range).
- Fit mapping in MATLAB/Python using templates.
- Implement compensation function + guard against division by zero.
- Mini demo: print distance in cm (and optionally a simple threshold message).

**Optional extension (only if ahead):** introduce smoothing/filtering (`IRSensor` class), compare raw vs averaged signal.

---

### Checkpoints

- After Block 0: repo forked/cloned/imported, target set, **build succeeds** (or error captured).
- After Block 1: LED toggles as expected with USER button; students can point to where the toggle logic lives.
- After Block 2: IR sensor model + valid range confirmed; live mV readings vary with distance; first valid data points recorded.
- After Block 3: calibration fit completed; mapping implemented in code; distance in cm printed; mini demo shown.

---

### Student announcement template (Teams)

**Subject:** “WS1 (Flipped) – do this before class”

- **Body:**
- **Before class (~110–155 min total):**
  - Read (required) — ~75–110 min total:
    - [README.md](../../README.md) (overview + safety, ~5–10 min)
    - [course_setup.md](course_setup.md) (tooling + Mbed Studio basics, ~15–20 min)
    - [tips.md](tips.md) (debugging + programming hygiene, ~10–15 min)
    - [ws1.md](ws1.md) (WS1 flow, ~8–12 min)
    - [main_description.md](main_description.md) (USER gating + structure of main, ~12–18 min)
    - [ir_sensor.md](ir_sensor.md) (range + wiring + calibration overview, ~25–35 min)
  - Quiz (required): https://forms.cloud.microsoft/e/knqQAypewF (QR in [ws1.md](ws1.md)). (~7–10 min)
  - Tools (required): install and open Mbed Studio once (per [course_setup.md](course_setup.md)). (~25–35 min)
    - Accounts: GitHub + Mbed.
    - Calibration: MATLAB (university login) or Python (numpy/matplotlib).

- **Bring to class:** laptop with Mbed Studio installed. (USB cable provided in class.)
- **In-class plan:** Block 0 setup/build → Part 1 LED + USER gating → IR wiring + data capture → fit mapping + mini demo.
- **Help rule:** if stuck >5 minutes in class, check hints in [ws1.md](ws1.md), then ask instructor.

---

## WS2

### Ordered reading (per README numbers)
1) [README.md](../../README.md)
2) [ws2.md](ws2.md)
3) [servo.md](servo.md)
4) [ultrasonic_sensor.md](ultrasonic_sensor.md)

Solutions:
- [../solutions/main_servo.cpp](../solutions/main_servo.cpp)
- [../solutions/main_ws2_p2.cpp](../solutions/main_ws2_p2.cpp)
- [../solutions/main_ws2_p2_ir_sensor.cpp](../solutions/main_ws2_p2_ir_sensor.cpp)

### Pre-class (read + quiz; tools not counted in time)

**Target time (reading/understanding only):** ~45–70 min total (students can split into two sessions).

**Read (same order as mapping, with estimated read/understand time):**
- README.md (overview + safety) — ~5–10 min
- ws2.md (WS2 flow) — ~10–15 min
- servo.md (servo wiring, calibration, control) — ~15–20 min
- ultrasonic_sensor.md (ultrasonic wiring/usage; IR fallback mention) — ~15–25 min

**Quiz (MS Forms, ~6–8 min total):**
- 6 auto-graded items.
- Focus: servo safety/calibration basics, pulse-width mapping, ultrasonic range/valid-read handling, state-machine role, and one catch-up question on README safety.

**Pre-class outputs:**
- Quiz completed.
- Mbed Studio opened once; GitHub + Mbed accounts ready.
- Know planned pins: servo(s) per servo.md; ultrasonic on PB_D3 (or IR fallback on PB_A0) per ws2.md/ultrasonic_sensor.md.

### WS2 Quiz Outline (MS Forms)

- 6 auto-graded items. Total quiz time ~6–8 min.

**Suggested questions (core 4)**
- Servo safety (single choice): “What must be true before reconnecting the servo wires?”
  Options: a) Power off/disconnect USB; b) Increase PWM frequency; c) Switch to 5V rail; d) Hold the horn.
  Answer: a.

- Pulse-width mapping (numeric): “Given 1.0 ms–2.0 ms limits, what pulse width corresponds to command = 0.25?”
  Expected: ~1.25 ms (accept ~1.22–1.28 ms).

- Calibration intent (single choice): “Why calibrate the servo endpoints?”
  Options: a) To change torque; b) To avoid mechanical stall and map normalized commands; c) To reduce CPU usage; d) To enable UART.
  Answer: b.

- Ultrasonic validity (single choice): “When `UltrasonicSensor::read()` returns an invalid value (e.g., -1.0f), what should you do?”
  Options: a) Use it anyway; b) Keep the last valid reading and skip the update; c) Multiply by zero; d) Stop the loop.
  Answer: b.

### In-class (4 × 45 min, same structure as WS1)

**Block 0 (Setup):** fork/clone/import, target NUCLEO-F446RE, build once. Output: build succeeds or error captured.

**Block 1 (Servo wiring + calibration + control):**
- Wire and calibrate one servo per servo.md; flash and sweep; confirm mapping 0.0–1.0 → pulse width → physical deflection.

**Block 2 (Ultrasonic wiring + live reads; IR fallback if needed):**
- Wire ultrasonic sensor to PB_D3; read/print cm values; handle invalid readings. If no ultrasonic available, use IR sensor via IRSensor class with existing calibration.

**Block 3 (State machine integration + demo):**
- Implement the WS2 state machine (Initial → Execution/Sleep/Emergency) mapping distance to servo input; include range checks and emergency on mechanical button. Mini demo: servo responds to distance and transitions correctly.

**Optional extension:** compare raw vs filtered distance (simple averaging) or add a “catch-up lane” role for unprepared students.

### Checkpoints

- After Block 0: repo imported, target set, **build succeeds** (or error captured).
- After Block 1: servo wired/calibrated; sweep works; students can point to the mapping line.
- After Block 2: distance readings change with motion; invalid-read handling confirmed (no crashes/NaN flow).
- After Block 3: state machine runs; servo responds to distance with defined ranges; emergency path tested.

### Student announcement template (Teams)

**Subject:** “WS2 (Flipped) – do this before class”

**Body:**
- **Before class (~45–70 min total for reading):**
  - Read (required):
    - [README.md](../../README.md) (overview + safety, ~5–10 min)
    - [ws2.md](ws2.md) (WS2 flow, ~10–15 min)
    - [servo.md](servo.md) (servo wiring/calibration/control, ~15–20 min)
    - [ultrasonic_sensor.md](ultrasonic_sensor.md) (ultrasonic wiring/usage; IR fallback, ~15–25 min)
  - Quiz (required, 6 questions): Link TBA; scan QR in [ws2_quiz_qr_code.png](../images/ws2_quiz_qr_code.png) or [ws2.md](ws2.md). (~6–8 min)

- **Bring to class:** laptop with Mbed Studio installed. (USB cable provided in class.)
- **In-class plan:** Block 0 setup/build → Block 1 servo wiring/calibration → Block 2 ultrasonic wiring + live reads → Block 3 state machine + demo.
- **Help rule:** if stuck >5 minutes, check hints in [ws2.md](ws2.md), then ask instructor.

---

## WS3

### Ordered reading (per README numbers)
1) [README.md](../../README.md)
2) [ws3.md](ws3.md)
3) [dc_motor.md](dc_motor.md)

Solutions:
- [../solutions/main_dc_motor.cpp](../solutions/main_dc_motor.cpp)
- [../solutions/main_ws3_p2.cpp](../solutions/main_ws3_p2.cpp)
- [../solutions/main_ws3_p2_ir_sensor.cpp](../solutions/main_ws3_p2_ir_sensor.cpp)

### Pre-class (read + quiz; tools not counted in time)

**Target time (reading/understanding only):** ~55–90 min total.

**Read (same order as mapping, with estimated read/understand time):**
- README.md (overview + safety + battery caution) — ~5–10 min
- ws3.md (WS3 flow) — ~12–18 min
- dc_motor.md (motor wiring, H-bridge/PWM basics, encoder direction, motion planner) — ~35–55 min

**Quiz (MS Forms, ~5–7 min total):**
- 4 auto-graded items.
- Focus: battery/power safety, H-bridge/PWM mapping, encoder direction awareness, and the WS3 state-machine emergency trigger.

**Pre-class outputs:**
- Quiz completed.
- Recall default pins: DC motor on M1 pins (PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1) with `PB_ENABLE_DCMOTORS`; ultrasonic on PB_D3; mechanical button on PC_5.
- Know battery setup: two packs → ~12 V; one pack → ~6 V (bridge remaining pins) per dc_motor.md; power off when rewiring.

### WS3 Quiz Outline (MS Forms)

- 4 auto-graded items. Total quiz time ~5–7 min.

**Suggested questions (core 4)**
- Battery safety (single choice): “When is it safe to reconnect the battery leads?” Options: a) While motor spins; b) Only when power switch off and wiring checked; c) Any time USB is plugged; d) When encoder is unplugged. Answer: b.

- H-bridge/PWM mapping (single choice): “What does a 0.6 duty command on PB_PWM_M1 do?” Options: a) Reverse full speed; b) Forward ~60% duty (direction per enable wiring); c) No effect; d) Turns off encoder. Answer: b.

- Encoder direction (single choice): “If commanded forward yields negative counts, what do you do?” Options: a) Ignore; b) Swap encoder channels or invert sign in software; c) Swap battery packs; d) Reduce duty. Answer: b.

- State-machine emergency (single choice): “What triggers the Emergency state in WS3?” Options: a) Timer overflow; b) Ultrasonic distance below threshold (e.g., 4.5 cm) during the forward stroke; c) Encoder zero; d) USB disconnect. Answer: b.

### In-class (4 × 45 min, same structure as WS1)

**Block 0 (Setup + safety):** confirm repo/import/target/build; power switch off while wiring; identify `PB_ENABLE_DCMOTORS` control.

**Block 1 (Motor bring-up + direction check):** wire motor on M1 pins; enable power electronics; run a low-duty open-loop or small velocity command; verify encoder sign matches command (swap A/B or invert if not); print rpm.

**Block 2 (Position control + motion planner):** enable motion planner; set limited acceleration; command small forward/back rotations; confirm position feedback behaves and stops; note gear ratio/setpoint meaning.

**Block 3 (State machine + emergency demo):** implement WS3 state machine (Initial → Sleep → Forward → Backward → Emergency) with ultrasonic guard (<~4.5 cm) and mechanical button stop; demo forward stroke, return, and emergency trigger.

### Checkpoints

- After Block 0: repo builds; safety rule observed (power off when rewiring); power electronics toggle tested.
- After Block 1: motor spins; encoder sign verified; rpm print is sensible.
- After Block 2: position command executes with bounded accel; forward/back motions repeatable.
- After Block 3: state machine runs; emergency triggers on obstacle/button; system returns to sleep after cycle.

### Student announcement template (Teams)

**Subject:** “WS3 (Flipped) – do this before class”

**Body:**
- **Before class (~55–90 min total for reading):**
  - Read (required):
    - [README.md](../../README.md) (overview + safety + battery cautions, ~5–10 min)
    - [ws3.md](ws3.md) (WS3 flow, ~12–18 min)
    - [dc_motor.md](dc_motor.md) (motor wiring/H-bridge/PWM/encoder/motion planner, ~35–55 min)
  - Quiz (required, 4 questions): Link TBA; scan QR in [ws3_quiz_qr_code.png](../images/ws3_quiz_qr_code.png) or [ws3.md](ws3.md). (~5–7 min)

- **Bring to class:** laptop with Mbed Studio installed; hardware kit. USB cable provided in class.
- **Pins:** motor on M1 pins (PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, `PB_ENABLE_DCMOTORS`); ultrasonic on PB_D3; mechanical button on PC_5.
- **In-class plan:** Block 0 build/safety → Block 1 motor bring-up + direction → Block 2 position control + planner → Block 3 state machine + emergency demo.
- **Help rule:** if stuck >5 minutes, check hints in [ws3.md](ws3.md), then ask instructor.
