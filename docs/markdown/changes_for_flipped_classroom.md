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

## Q2 (done in class)
Why is a flow chart created before coding a robot task?

- A) It removes the need for testing
- B) It helps define states/transitions and catch logic issues early
- C) It guarantees bug-free implementation

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

## Q5 (in class)
`AnalogIn.read()` returns `0.0...1.0`. What converts this to millivolts as used in the workshop?

- A) `ir_mV = read()*1000`
- B) `ir_mV = read()*3.3*1000`
- C) `ir_mV = read()/3.3*1000`

## Q6
Why must calibration points stay within the specific sensor’s valid range (and be denser near minimum distance)?

- A) Because outside-range readings can saturate or be undefined, and the response is steep near the minimum so you need more points for an accurate mapping
- B) Because `AnalogIn` stops working above 2.0 V, so only in-range points are readable
- C) Because the analog readout is most stable in the middle of the range, so edge samples should be avoided
- D) Because Mbed Studio clamps out-of-range values to 0.0 automatically during calibration

## Q7
The IR sensor has a valid range from 12 cm to 60 cm. During calibration, at 12 cm the sensor reads 2850 mV, and at 60 cm it reads 850 mV. A student creates a linear mapping function. If the sensor currently reads 2050 mV, what distance (in cm) does the mapping function return?

- A) ~19 cm
- B) ~25 cm
- C) ~31 cm
- D) ~44 cm

## Q8
To reduce noise, a student averages three consecutive IR sensor readings: 2050 mV, 2045 mV, 2055 mV (read at slightly different distances). When the average is passed through the linear calibration mapping from Q7, what distance (in cm) is returned?

- A) ~27 cm
- B) ~29 cm
- C) ~31 cm
- D) ~33 cm

---

## Answer Key

- Q1: **B**
- Q2: **B**
- Q3: **B**
- Q4: **C**
- Q5: **B**
- Q6: **A**
- Q7: **C**
- Q8: **C**

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

## Q3 (in class)
What is the purpose of servo calibration (`calibratePulseMinMax`) before normal operation?

- A) It sets servo-specific safe min/max pulse mapping so normalized commands are meaningful
- B) It converts servo commands directly to degrees
- C) It compensates for gearbox backlash so midpoints are mechanically precise

## Q4 (in class)
In the ultrasonic driver workflow, what does `us_sensor.read()` returning `-1.0f` mean?

- A) The object is exactly at -1 cm
- B) The sensor is still settling after a trigger and is not ready to report yet
- C) No new valid measurement is currently available

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
- C) Faster polling shortens the echo timeout window so late echoes are missed
- D) Faster polling forces the servo into emergency mode

## Q7
A servo pulse-width mapping is calibrated: 1.0 ms pulse = 0°, 2.0 ms pulse = 180°. The servo command range is 0.0 (fully counterclockwise) to 1.0 (fully clockwise), and this is mapped linearly to pulse width. If you command 0.4 to the servo, what pulse width (in milliseconds) is approximately sent to the servo?

- A) 1.2 ms
- B) 1.4 ms
- C) 1.6 ms
- D) 1.8 ms

## Q8
The ultrasonic sensor requires at least 12 milliseconds between successive valid readings. The WS2 state machine loop runs at 50 Hz (20 ms per cycle). If you poll the ultrasonic sensor once per cycle, how many **complete valid measurements** can you reliably obtain per second?

- A) ~40 measurements/sec
- B) ~50 measurements/sec
- C) ~80 measurements/sec
- D) ~100 measurements/sec

---

## Answer Key

- Q1: **A**
- Q2: **B**
- Q3: **A**
- Q4: **C**
- Q5: **C**
- Q6: **B**
- Q7: **B**
- Q8: **B**

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

   ## Q1
   You are preparing your WS3 setup. Which step is required to let the DC motor actually receive power from the PES board?

   - A) Call `motor_M3.enableMotionPlanner();`
   - B) Set `enable_motors = 1;` using `DigitalOut enable_motors(PB_ENABLE_DCMOTORS);`
   - C) Call `motor_M3.setRotation(0.0f);`
   - D) Set `servo_D0.enable();`

   ## Q2
   In WS3, which setup correctly enables closed-loop position control on M1?

   - A) `FastPWM pwm_M1(PB_PWM_M1);` with encoder reads handled manually in the loop
   - B) `DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);`
   - C) `DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1);` and enable motors only
   - D) `DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1);` and call `setRotation()` only

   ## Q3
   During testing, you command a positive motion, but measured rotation goes negative. What are the most likely issues?

   - A) Motor leads are reversed
   - B) Encoder direction and motor direction are inconsistent with sign convention
   - C) Motion planner acceleration is too low
   - D) `printf()` is too slow
   - E) Encoder channels A and B are swapped

   ## Q4 (in class)
   You want smoother, bounded-acceleration motion before running the can-crusher state machine. What should you do?

   - A) Enable the motion planner and reduce the main loop period
   - B) Enable the motion planner and limit max acceleration
   - C) Clamp the target rotation range to a smaller interval

   ## Q5
   In WS3 FORWARD state, which condition should send the system to EMERGENCY?

   - A) `motor_M3.getRotation() > 1.49f`
   - B) `mechanical_button.read() == 0`
   - C) `us_distance_cm < 4.5f`
   - D) `us_distance_cm > 4.5f`

   ## Q6 (in class)
   After an EMERGENCY stop and USER-button reset, why do we reset motion-planner internal state (position/velocity) and re-enable it?

   - A) To avoid stale planner states and ensure predictable next motion cycle
   - B) To re-zero the encoder count to prevent wraparound
   - C) To increase the PWM frequency for smoother motion

   ## Q7
   A 78:1 geared DC motor with encoder has 48 CPR (counts per revolution) at the motor shaft. Including the gear reduction, how many encoder counts occur for one complete rotation of the output shaft (driven load)?

   - A) 576 counts
   - B) 1920 counts
   - C) 3744 counts
   - D) 7488 counts

   ## Q8
   The 78:1 geared DC motor has a nominal no-load speed of 345 RPM at the motor shaft. What is the approximate maximum output shaft speed in RPM after the gear reduction?

   - A) ~4.4 RPM
   - B) ~8.8 RPM
   - C) ~44 RPM
   - D) ~88 RPM

## Answer Key

- Q1: **B**
- Q2: **B**
- Q3: **A, B, E**
- Q4: **B**
- Q5: **C**
- Q6: **A**
- Q7: **C**
- Q8: **A**

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

---

## WS4

### Ordered reading (per README numbers)
1) [README.md](../../README.md)
2) [ws4.md](ws4.md)
3) [dd_kinematics.md](dd_kinematics.md)
4) [line_follower.md](line_follower.md)

Solutions:
- [../solutions/main_dd_kinematic_calib.cpp](../solutions/main_dd_kinematic_calib.cpp)
- [../solutions/main_line_follower_base.cpp](../solutions/main_line_follower_base.cpp)
- [../solutions/main_line_follower.cpp](../solutions/main_line_follower.cpp)

### Pre-class (read + quiz; tools not counted in time)

**Target time (reading/understanding only):** ~55–90 min total (students can split into two sessions).

**Read (same order as mapping, with estimated read/understand time):**
- README.md (overview + safety + battery caution) — ~5–10 min
- ws4.md (WS4 flow) — ~10–15 min
- dd_kinematics.md (differential drive kinematics, derivation, Eigen implementation) — ~20–30 min
- line_follower.md (sensor wiring, SensorBar/LineFollower driver, proportional control law) — ~20–30 min

**Quiz (MS Forms, ~7–10 min total):**
- 8 auto-graded items.
- Focus: sensor wiring safety, I2C pin assignment, kinematic transformation matrix direction, code patterns for wheel-speed conversion, and two numerical inverse-kinematics calculations.

**Pre-class outputs:**
- Quiz completed.
- Recall key pins: motors M1 on (PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1) and M2 on (PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2), `PB_ENABLE_DCMOTORS`; line follower I2C on PB_8 (SCL) / PB_9 (SDA) with 5 V supply.
- Know that `Cwheel2robot` maps wheel → robot velocities, and its inverse maps robot → wheel velocities.

### WS4 Quiz Outline (MS Forms)

- 8 auto-graded items. Total quiz time ~7–10 min.

## Workshop 4 – Pre-class Quiz (8 MC Questions)

   ## Q1
   Before connecting the SparkFun Line Follower Array to the PES board, what is the most critical precaution?

   - A) Flash a test firmware to verify the I2C address before applying power
   - B) Set the I2C clock frequency to 400 kHz in software before making any hardware connections
   - C) Carefully review the provided wiring images; incorrect connections can permanently destroy the sensor
   - D) Always use SPI instead of I2C for the line follower array to avoid bus collisions

   ## Q2
   The SparkFun Line Follower Array communicates via I2C. Which Nucleo pins are used for SCL and SDA?

   - A) SCL: PB_9, SDA: PB_8
   - B) SCL: PB_8, SDA: PB_9
   - C) SCL: PB_D3, SDA: PB_D2
   - D) SCL: PA_5, SDA: PA_7

   ## Q3
   Which of the following statements about the differential drive kinematic model are correct?

   - A) `Cwheel2robot` maps wheel angular velocities (ω1, ω2) to robot velocities (v, ω)
   - B) `Cwheel2robot` maps robot velocities (v, ω) to wheel angular velocities (ω1, ω2)
   - C) To compute wheel speeds from desired robot velocities, apply `Cwheel2robot.inverse() * robot_coord`
   - D) A positive robot angular velocity ω (counter-clockwise viewed from above) requires the right wheel (ω1) to rotate faster than the left wheel (ω2)
   - E) Wheel speeds from the Eigen result are passed directly to `motor.setVelocity()` without any unit conversion

   ## Q4 (in class)
   When using the `LineFollower` driver class for closed-loop line following, should the DC motor motion planner be enabled?

   - A) Yes, always enable the motion planner for smoother, bounded-acceleration motion
   - B) No, the motion planner should be disabled (default) for best line-following performance
   - C) Only enable it when the line track includes sharp 90° turns

   ## Q5
   The `isAnyLedActive()` check guards the angle update in:
   ```cpp
   if (sensor_bar.isAnyLedActive())
       angle = sensor_bar.getAvgAngleRad();
   ```
   Why is this guard necessary?

   - A) To prevent the sensor bar from overheating during continuous I2C reads
   - B) To avoid overwriting the last valid angle with an undefined value when no line is detected beneath any sensor
   - C) To limit the I2C polling rate to the sensor's minimum 3.2 ms read cycle
   - D) To verify that the 5 V supply voltage is within specification before reading

   ## Q6 (in class)
   In the simple proportional control law `robot_coord(1) = Kp * angle`, what is the effect of increasing `Kp`?

   - A) The robot reacts more aggressively to angular deviations from the line; a value that is too large will cause oscillations
   - B) The robot's maximum forward speed increases proportionally with Kp
   - C) The measurement range of the line follower sensor bar increases

   ## Q7
   A differential drive robot has wheel radius r = 0.025 m and wheelbase b = 0.13 m. The desired robot velocity is v = 0.5 m/s and ω = 0 rad/s. Applying the inverse kinematics, what are the right (ω1) and left (ω2) wheel angular speeds in rad/s?

   - A) ω1 = 10 rad/s, ω2 = 10 rad/s
   - B) ω1 = 20 rad/s, ω2 = 20 rad/s
   - C) ω1 = 20 rad/s, ω2 = −20 rad/s
   - D) ω1 = 2.6 rad/s, ω2 = 2.6 rad/s

   ## Q8
   Same robot (r = 0.025 m, b = 0.13 m). Desired: v = 0 m/s, ω = 1.0 rad/s (rotate in place, counter-clockwise from above). Applying the inverse kinematics, what are ω1 and ω2?

   - A) ω1 = 1.0 rad/s, ω2 = −1.0 rad/s
   - B) ω1 = 5.2 rad/s, ω2 = −5.2 rad/s
   - C) ω1 = 2.6 rad/s, ω2 = −2.6 rad/s
   - D) ω1 = 2.6 rad/s, ω2 = 2.6 rad/s

## Answer Key

- Q1: **C**
- Q2: **B**
- Q3: **A, C, D**
- Q4: **B**
- Q5: **B**
- Q6: **A**
- Q7: **B**
- Q8: **C**

### In-class (4 × 45 min, same structure as WS1–WS3)

**Block 0 (Setup + safety):** confirm repo/import/target/build; review line follower wiring guide before touching hardware; identify `PB_ENABLE_DCMOTORS` toggle.

**Block 1 (DC motors + kinematic bring-up):** wire motors M1 and M2; enable power electronics; implement `Cwheel2robot` and its inverse in C++ using Eigen; send velocity commands; print wheel and robot velocities and verify direction signs.

**Block 2 (Line follower connection + sensor readings):** connect sensor array via I2C (PB_8/PB_9 + 5 V); print raw bit values and angle; move sensor over a black line and confirm angle changes correctly; verify `isAnyLedActive()` guard behavior.

**Block 3 (Full line following + tuning):** combine kinematic transform with proportional angle controller; tune `Kp` until the robot follows the line without oscillating; demo: robot autonomously tracks a line. Optional: implement nonlinear Kp or forward-velocity modulation based on angle magnitude.

### Checkpoints

- After Block 0: repo builds; line follower wiring guide reviewed; power safety observed before connecting sensor.
- After Block 1: both motors spin in the correct direction; kinematic transform implemented; printed robot/wheel velocities are consistent.
- After Block 2: sensor connected without damage; angle reading changes as sensor crosses line; `isAnyLedActive()` confirmed to prevent stale angle overwrites.
- After Block 3: robot follows line; `Kp` tuned to a stable response; demo completed.

### Student announcement template (Teams)

**Subject:** "WS4 (Flipped) – do this before class"

**Body:**
- **Before class (~55–90 min total for reading):**
  - Read (required):
    - [README.md](../../README.md) (overview + safety + battery cautions, ~5–10 min)
    - [ws4.md](ws4.md) (WS4 flow, ~10–15 min)
    - [dd_kinematics.md](dd_kinematics.md) (differential drive kinematics, derivation + Eigen code, ~20–30 min)
    - [line_follower.md](line_follower.md) (sensor wiring, SensorBar/LineFollower driver, control law, ~20–30 min)
  - Quiz (required, 8 questions): Link TBA; scan QR in [ws4.md](ws4.md). (~7–10 min)

- **Bring to class:** laptop with Mbed Studio installed; fully assembled differential drive robot kit.
- **Pins:** motors M1 on (PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1) and M2 on (PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2), `PB_ENABLE_DCMOTORS`; line follower I2C on PB_8 (SCL) and PB_9 (SDA) with 5 V supply.
- **In-class plan:** Block 0 build/safety → Block 1 motors + kinematics → Block 2 line follower connection + readings → Block 3 line following + tuning demo.
- **Help rule:** if stuck >5 minutes, check hints in [ws4.md](ws4.md) and [line_follower.md](line_follower.md), then ask instructor.

---

## WS5

### Ordered reading (per README numbers)
1) [README.md](../../README.md)
2) [ws5.md](ws5.md)
3) [imu.md](imu.md)
4) [serial_stream.md](serial_stream.md)

Solutions:
- [../solutions/main_gimbal.cpp](../solutions/main_gimbal.cpp)
- [../solutions/main_gimbal_1d_mahony.cpp](../solutions/main_gimbal_1d_mahony.cpp)
- [../solutions/main_gimbal_3d_mahony.cpp](../solutions/main_gimbal_3d_mahony.cpp)
- [../solutions/main_comp_filter.cpp](../solutions/main_comp_filter.cpp)
- [../solutions/matlab/serial_stream_comp_filter.m](../solutions/matlab/serial_stream_comp_filter.m)
- [../solutions/python/serial_stream_comp_filter.py](../solutions/python/serial_stream_comp_filter.py)

### Pre-class (read + quiz; tools not counted in time)

**Target time (reading/understanding only):** ~40–60 min total (students can split into two sessions).

**Read (same order as mapping, with estimated read/understand time):**
- README.md (overview + safety) — ~5–10 min
- ws5.md (WS5 flow) — ~8–12 min
- imu.md (IMU principle, gyro/acc/fusion overview, complementary filter, gimbal wiring) — ~15–25 min
- serial_stream.md (UART wiring, TX/RX pins, start byte, send/write API) — ~10–15 min

**Quiz (MS Forms, ~7–10 min total):**
- 8 auto-graded items.
- Focus: gyroscope vs accelerometer characteristics, complementary filter role of each sensor, 1-D Mahony filter structure, serial stream wiring, and two short calculation questions on filter discretization and angle mapping.

**Pre-class outputs:**
- Quiz completed.
- Recall key pins: IMU I2C on PC_9 (SDA) / PA_8 (SCL); SerialStream TX: PB_10, RX: PC_5; servos roll on PB_D0, pitch on PB_D1.
- Know that `imu_data.pry(1)` is roll and `imu_data.pry(0)` is pitch in the ZXY Tait-Bryan convention.

### WS5 Quiz Outline (MS Forms)

- 8 auto-graded items. Total quiz time ~7–10 min.

## Workshop 5 – Pre-class Quiz (8 MC Questions)

   ## Q1
   The gyroscope signal is modelled as $y_{gyro}(t) = \omega_x(t) + b_{gyro}(t)$. Why can the gyroscope NOT be used alone to reliably estimate an angle over a long period of time?

   - A) The gyroscope only measures linear acceleration, not angular velocity
   - B) Integrating the gyroscope signal accumulates the bias over time, causing the angle estimate to drift
   - C) The gyroscope has too much high-frequency noise, making angle integration useless
   - D) The gyroscope saturates after 90° of rotation

   ## Q2
   The accelerometer-based angle estimate is $y_{acc} = \phi + \eta_{acc}$. What is the main limitation of using the accelerometer alone for angle estimation?

   - A) The accelerometer introduces a slowly varying bias that is impossible to correct
   - B) The accelerometer can only measure angles around the Z-axis
   - C) The accelerometer signal is corrupted by significant noise and is also sensitive to linear accelerations of the platform
   - D) The accelerometer cannot measure angles less than 45°

   ## Q3
   In the complementary filter, the accelerometer measurement passes through a low-pass filter and the gyroscope integral passes through a high-pass filter. What is the motivation for this combination?

   - A) The low-pass filter removes the gyroscope drift, while the high-pass filter removes sensor noise
   - B) The gyroscope is reliable in the short term (high-frequency), while the accelerometer is reliable in the long term (low-frequency), so each signal contributes where it is most accurate
   - C) The high-pass filter doubles the accelerometer bandwidth, and the low-pass filter reduces the gyroscope update rate to save processing power
   - D) Both filters are applied only to remove electromagnetic interference from the motor drivers

   ## Q4 (in class)
   Calibrating the servos with `calibratePulseMinMax(servo_ang_min, servo_ang_max)` before commanding the gimbal is required. Why?

   - A) It resets the servo position to mechanical zero
   - B) It maps the normalized command range (0.0–1.0) to the servo-specific safe min/max pulse widths so that angle commands are physically meaningful
   - C) It sets the PWM carrier frequency to match the servo's internal oscillator

   ## Q5
   The `SerialStream` TX and RX pin labels can be confusing when connecting the USB Serial TTL cable. Which connection is correct?

   - A) TX of the cable to PB_10 (microcontroller TX), RX of the cable to PC_5 (microcontroller RX)
   - B) TX of the cable to PC_5 (microcontroller RX), RX of the cable to PB_10 (microcontroller TX)
   - C) Both TX and RX of the cable to PB_10 because the pin is bidirectional
   - D) TX of the cable to any available GPIO, polarity does not matter for UART

   ## Q6 (in class)
   In the gimbal code, `rp(0) = imu_data.pry(1)` is used for roll and `rp(1) = imu_data.pry(0)` for pitch. Why are the ZXY Tait-Bryan angles used (`.pry`) instead of the ZYX angles (`.rpy`)?

   - A) The ZYX convention has a gimbal-lock singularity at pitch ±90°, whereas ZXY has the singularity at roll ±90°; the chosen convention matches the physical range of the gimbal
   - B) `.pry` is the only convention that includes magnetometer data
   - C) The ZYX convention is not supported by the Mahony filter implementation in the IMU driver

   ## Q7
   The 1-D Mahony filter discrete update is:
   $$\phi_{est}[k] = \phi_{est}[k-1] + T_s\,(y_{gyro,x}[k] + k_p\,(y_{acc,x}[k] - \phi_{est}[k-1]))$$
   With $k_p = 2.0$, $T_s = 0.02$ s, $\phi_{est}[k-1] = 0.10$ rad, $y_{gyro,x}[k] = 0.0$ rad/s, $y_{acc,x}[k] = 0.20$ rad. What is $\phi_{est}[k]$?

   - A) 0.102 rad
   - B) 0.104 rad
   - C) 0.110 rad
   - D) 0.120 rad

   ## Q8
   The servo command is mapped from an angle $\phi$ to a normalized pulse width using:
   `servo_width = -normalised_angle_gain * φ + normalised_angle_offset`
   where `normalised_angle_gain = 1/π` and `normalised_angle_offset = 0.5`. If the measured roll angle is $\phi = -\pi/4$ rad (−45°), what normalized pulse width is sent to the roll servo?

   - A) 0.25
   - B) 0.50
   - C) 0.75
   - D) 1.00

## Answer Key

- Q1: **B**
- Q2: **C**
- Q3: **B**
- Q4: **B**
- Q5: **B**
- Q6: **A**
- Q7: **B**
- Q8: **C**

### In-class (4 × 45 min, same structure as WS1–WS4)

**Block 0 (Setup + safety):** confirm repo/import/target/build; calibrate servos (roll on PB_D0, pitch on PB_D1) before powering the gimbal; verify IMU readings print sensibly.

**Block 1 (IMU bring-up + gimbal control):** create `IMU` and `Servo` objects; read roll and pitch from `imu_data.pry`; map angles to servo commands; verify gimbal physically compensates for board tilt.

**Block 2 (Serial Streaming setup + data capture):** wire USB Serial TTL cable (TX→PC_5, RX→PB_10 + GND); flash `main_comp_filter.cpp`; confirm data reception in MATLAB or Python; capture a tilt sequence for offline analysis.

**Block 3 (Complementary / Mahony filter implementation + demo):** implement complementary filter (or 1-D Mahony) in MATLAB or Python on captured data; compare estimate with built-in IMU output; port working algorithm to C++ and integrate into gimbal control; demo: gimbal stabilizes using student's own filter.

### Checkpoints

- After Block 0: repo builds; servos calibrated; printed roll/pitch values change correctly when board is tilted.
- After Block 1: gimbal compensates tilt in both axes; angle limits guard prevents servo overrun.
- After Block 2: serial stream data received on host; delta-time column is plausible (~20 ms per sample at 50 Hz).
- After Block 3: complementary or Mahony filter implemented and verified offline; C++ port runs on board; gimbal uses student's own angle estimate.

### Student announcement template (Teams)

**Subject:** "WS5 (Flipped) – do this before class"

**Body:**
- **Before class (~40–60 min total for reading):**
  - Read (required):
    - [README.md](../../README.md) (overview + safety, ~5–10 min)
    - [ws5.md](ws5.md) (WS5 flow, ~8–12 min)
    - [imu.md](imu.md) (IMU principle, complementary filter, gimbal code, ~15–25 min)
    - [serial_stream.md](serial_stream.md) (UART wiring, TX/RX pins, API usage, ~10–15 min)
  - Quiz (required, 8 questions): Link TBA; scan QR in [ws5_quiz_qr_code.png](../images/ws5_quiz_qr_code.png) or [ws5.md](ws5.md). (~7–10 min)

- **Bring to class:** laptop with Mbed Studio installed + MATLAB or Python (numpy/matplotlib) for offline analysis; assembled 2-axis gimbal kit; USB Serial TTL cable.
- **Pins:** IMU I2C on PC_9 (SDA) / PA_8 (SCL) — on-board, no wiring needed; servos roll on PB_D0, pitch on PB_D1; SerialStream TX: PB_10, RX: PC_5.
- **In-class plan:** Block 0 build/safety + servo calibration → Block 1 IMU bring-up + gimbal control → Block 2 serial streaming + data capture → Block 3 filter implementation + demo.
- **Help rule:** if stuck >5 minutes, check hints in [ws5.md](ws5.md) and [imu.md](imu.md), then ask instructor.

---

## Post-course Student Feedback Survey

Thank you for completing this post-course feedback survey. Its purpose is to evaluate your experience with the flipped-format workshops (WS1–WS3) and help improve the course for future students. Your responses will be summarized anonymously, so please answer honestly and based on your own experience.

**Likert scale (1-5):**
1 = Strongly disagree, 2 = Disagree, 3 = Neutral, 4 = Agree, 5 = Strongly agree

### Likert items

1) I participated in all three flipped-format workshops (WS1–WS3).
2) The pre-class materials were clear and concise.
3) The time required for pre-class work was reasonable each week.
4) The pre-class work prepared me to complete the in-class tasks.
5) In-class time was used effectively for hands-on practice.
6) Instructor/TA feedback during class was timely and helpful.
7) The workshop instructions were clear enough to follow independently.
8) The provided examples/solutions helped me debug and learn.
9) The pacing across WS1-WS3 was appropriate.
10) I feel confident to be able to complete similar tasks on my own after this course for our robotics project.

### Open-ended items

11) What was the most valuable part of the flipped format for you?
12) What was the biggest friction point (prep, tooling, hardware, or in-class)?
13) If you could change one thing for next year, what would it be?
14) What should we definitely keep as-is?
