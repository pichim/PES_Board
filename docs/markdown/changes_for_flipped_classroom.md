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

# Workshop 1 – Pre-Class Quiz (Q1–Q6)

## Q1
Before changing wiring on the PES + Nucleo setup, what is the recommended state?

- A) USB connected, PES switch ON, charger disconnected  
- B) All power sources disconnected, PES switch OFF  
- C) Only stop code execution in Mbed Studio  
- D) Keep board powered to verify live wiring  

## Q2
Why is a flow chart created before coding a robot task?

- A) It removes the need for testing  
- B) It helps define states/transitions and catch logic issues early  
- C) It guarantees bug-free implementation  
- D) It is only useful for reports, not coding  

## Q3
What is the role of the USER button callback `toggle_do_execute_main_fcn()`?

- A) It recalibrates the IR sensor each press  
- B) It toggles whether the main task executes (`do_execute_main_task`)  
- C) It changes the target board in Mbed Studio  
- D) It forces `while(true)` to stop  

## Q4
If `main_task_period_ms = 20`, what is the intended loop rate?

- A) 20 Hz  
- B) 25 Hz  
- C) 50 Hz  
- D) 100 Hz  

## Q5
`AnalogIn.read()` returns `0.0...1.0`. What converts this to millivolts as used in the workshop?

- A) `ir_mV = read()*1000`  
- B) `ir_mV = read()*5.0*1000`  
- C) `ir_mV = read()*3.3*1000`  
- D) `ir_mV = read()/3.3*1000`  

## Q6
Why must calibration points stay within the specific sensor’s valid range (and be denser near minimum distance)?

- A) Because outside-range behavior and near-minimum nonlinearity reduce mapping accuracy  
- B) Because Mbed cannot print values outside the range  
- C) Because the USER button only works within that range  
- D) Because calibration only works in Release build profile  

---

## Answer Key

- Q1: **B**  
- Q2: **B**  
- Q3: **B**  
- Q4: **C**  
- Q5: **C**  
- Q6: **A**

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

# Workshop 2 – Pre-Class Quiz (Q1–Q6)

## Q1
In the WS2 intended state-machine logic, what transition should happen from `EXECUTION`?

- A) Out-of-range distance → `SLEEP`; mechanical button event → `EMERGENCY`  
- B) Out-of-range distance → `EMERGENCY`; mechanical button event → `SLEEP`  
- C) Any invalid read → `INITIAL`  
- D) Mechanical button event → `INITIAL` directly  

## Q2
The mechanical button is wired to `PC_5` and `GND`, and the input uses `PullUp`. Which condition correctly detects a pressed button?

- A) `if (mechanical_button.read())` because pressed = HIGH  
- B) `if (!mechanical_button.read())` because pressed pulls the pin LOW  
- C) `if (mechanical_button.read() > 0.5f)` because it is analog  
- D) `if (mechanical_button == BUTTON1)` because `PC_5` mirrors USER button  

## Q3
What is the purpose of servo calibration (`calibratePulseMinMax`) before normal operation?

- A) It removes the need to enable the servo  
- B) It sets servo-specific safe min/max pulse mapping so normalized commands are meaningful  
- C) It converts servo commands directly to degrees  
- D) It increases USB flashing speed  

## Q4
In the ultrasonic driver workflow, what does `us_sensor.read()` returning `-1.0f` mean?

- A) The object is exactly at -1 cm  
- B) Sensor calibration is complete  
- C) No new valid measurement is currently available  
- D) The sensor must be power-cycled immediately  

## Q5
What is the recommended handling for ultrasonic invalid reads in WS2?

- A) Always assign the new value, including `-1.0f`  
- B) Assign `0.0f` whenever `-1.0f` appears  
- C) Update `us_distance_cm` only when the candidate read is valid (`> 0.0f`)  
- D) Stop the state machine on the first invalid read  

## Q6
Why should the ultrasonic sensor not be polled faster than about every `12000 µs` (as documented)?

- A) Faster polling increases PWM resolution  
- B) Faster polling causes frequent invalid readings (`-1.0f`)  
- C) Faster polling disables `PullUp` mode  
- D) Faster polling forces the servo into emergency mode  

---

## Answer Key

- Q1: **A**  
- Q2: **B**  
- Q3: **B**  
- Q4: **C**  
- Q5: **C**  
- Q6: **B**

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

## Workshop 3 – Pre-class Quiz (6 MC Questions)

1. You are preparing your WS3 setup. Which step is required to let the DC motor actually receive power from the PES board? 
   A) Call `motor_M3.enableMotionPlanner();`  
   B) Set `enable_motors = 1;` using `DigitalOut enable_motors(PB_ENABLE_DCMOTORS);`  
   C) Call `motor_M3.setRotation(0.0f);`  
   D) Set `servo_D0.enable();`

2. You are using one 78:1 motor on M1 pins for WS3 position control. Which object setup is the correct closed-loop approach?

   A) `FastPWM pwm_M1(PB_PWM_M1);` and command only with `write()`  
   B) `DCMotor motor_M3(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);`  
   C) `DigitalOut motor_M3(PB_PWM_M1);` with `motor_M3 = 1;`  
   D) `AnalogIn motor_M3(PB_PWM_M1);`

3. During testing, you command a positive motion, but measured rotation goes negative. What is the most likely issue?

   A) The ultrasonic sensor is unplugged  
   B) Encoder direction and motor direction are inconsistent with sign convention  
   C) Motion planner acceleration is too low  
   D) `printf()` is too slow

4. You want smoother, bounded-acceleration motion before running the can-crusher state machine. What should you do?  

   A) Disable the encoder  
   B) Enable the motion planner and limit max velocity  
   C) Enable the motion planner and limit max acceleration  
   D) Use open-loop PWM only

5. In WS3 FORWARD state, which condition should send the system to EMERGENCY? 
   
   A) `motor_M3.getRotation() > 2.89f`  
   B) `mechanical_button.read() == 0`  
   C) `us_distance_cm < 4.5f`  
   D) `us_distance_cm > 4.5f`

6. After an EMERGENCY stop and USER-button reset, why do we reset motion-planner internal state (position/velocity) and re-enable it?  
   
   A) To make the serial port faster  
   B) To avoid stale planner states and ensure predictable next motion cycle  
   C) To recalibrate the ultrasonic sensor  
   D) To reverse motor polarity automatically

## Answer Key

- Q1: **B**  
- Q2: **B**  
- Q3: **B**  
- Q4: **C**  
- Q5: **C**  
- Q6: **B**

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
