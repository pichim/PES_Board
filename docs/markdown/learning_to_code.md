# Learning to Code

Learning to program means being able to explain code, predict its behaviour and make a small change yourself. AI can help with explanations, feedback and debugging. Check what you can do after the conversation ends.

This is optional study advice for the PES Board course. Follow the workshop instructions and the AI permissions and disclosure rules communicated for each assessment. The activities below add no assessment or submission requirement. You can use the same approach with documentation, classmates and teaching assistants; no paid AI tool is needed.

## Finishing a Task and Learning From It

An AI assistant may produce a complete, correct solution. That does not establish what you have learned. Even if a generated IR sensor program prints correct distances, you may still be unable to explain the units, identify a calibration mistake or adapt the program yourself. Correctness still needs verification; AI output is not guaranteed to work.

Decide whether your current goal is to learn a new skill or automate a task you already understand. For a learning task, use assistance to practise the reasoning you want to acquire: make a prediction, work through an explanation, test it and try another case yourself. A complete example can also be useful study material if you actively investigate and modify it. Judge your progress by what you can explain and do afterwards, rather than only by whether the task is finished.

## Start With One Small Learning Goal

Use this routine with a short section of your current workshop:

1. **Read and predict.** Describe the inputs, outputs and expected behaviour. For example: “This value should be in millivolts after the conversion.”
2. **Try a step yourself.** Trace a value, sketch the states or write a small part of your approach. Break a larger task into steps before implementing it.
3. **Get focused help.** Show your attempt and ask for one hint or feedback on your reasoning. If you cannot start, ask for a simpler worked example and connect it to your task.
4. **Check the result.** Compare it with the supplied code and documentation. Write down an expected result before testing a change.
5. **Explain and revisit.** Close the answer, explain the idea in your own words and try a small related case. Revisit it in another study session and correct any gaps.

Ask for help when you need it; there is no required time to struggle alone. As a concept becomes familiar, try it with less assistance. Looking up an API is useful; simply retyping a generated solution does not show that you understand it.

When experimenting, preserve a working version. Follow [Working With Examples](../../README.md#working-with-examples) to save your current work and select an example for `src/main.cpp`. Keep only one application `main()` among the files compiled from `src`.

## Choose How Much Help to Use

| Help available                                                                                  | Use it deliberately                                                                                                                                                    |
| ----------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Browser chat**, such as ChatGPT                                                               | Paste or attach the relevant code, your attempt, the exact error or observation and your environment. Naming a file alone does not provide its contents.               |
| **Inline suggestions**, such as Copilot completions                                             | Predict what a suggestion will do before accepting it. Consider pausing automatic suggestions while practising a new concept.                                          |
| **Chat beside your code**                                                                       | Select or name the relevant files. Ask for an explanation or feedback and check which context the tool actually used.                                                  |
| **An agent that can edit files or run commands**, such as Codex or Copilot in an agent workflow | For a learning question, start with explanation only. If you request an edit, save a starting point, limit the scope, inspect the complete diff and verify the result. |

The same product can offer several of these functions. A prompt saying “do not edit” expresses your intention; it does not restrict access. Check the active tools and permissions. Product interfaces change; consult the official [Copilot Learning Guide](https://docs.github.com/en/get-started/learning-to-code/setting-up-copilot-for-learning-to-code), [Codex IDE Documentation](https://learn.chatgpt.com/docs/codex/ide) or [VS Code Agent Documentation](https://code.visualstudio.com/docs/agents/overview) when needed. Preserve the repository's existing editor configuration when adjusting personal learning preferences.

## Example 1: Trace an IR Sensor Value

In [Workshop 1](ws1.md), the [IR Sensor Example](../solutions/main_ir_sensor.cpp) contains:

```cpp
ir_distance_mV = 1.0e3f * ir_analog_in.read() * 3.3f;
```

Suppose `ir_analog_in.read()` returns `0.5f`. Before opening the explanation, calculate `ir_distance_mV` and identify its unit. This is a calculation on paper; you do not need to change the firmware or sensor wiring.

<details>
<summary>Check Your Reasoning</summary>

Using the example's 3.3 V scaling: `1000 × 0.5 × 3.3 = 1650 mV`. The normalized input has been converted to voltage. A distance in centimetres still needs the calibration mapping. The variable name alone does not establish what physical quantity it contains.

</details>

**Ask for help with your reasoning:**

> I am tracing the voltage conversion in `docs/solutions/main_ir_sensor.cpp`. Here is my calculation and its units: [my attempt]. Check my reasoning and give one hint if needed. Do not complete the calibration task or edit files.

Next, inspect the [IRSensor Header](../../lib/IRSensor/IRSensor.h) and [Implementation](../../lib/IRSensor/IRSensor.cpp). `readmV()` returns the stored voltage in millivolts. After calibration has been configured and the readings updated, `readcm()` returns the calibrated value in centimetres, and `read()` returns its filtered value. Without calibration, even `readcm()` contains the millivolt value. Explain that distinction before choosing a method. For real measurements, use your sensor's calibration and valid range from the [IR Sensor Tutorial](ir_sensor.md).

**Try without the answer:** repeat the voltage calculation for a normalized input of `0.25f`, then check the arithmetic. For calibration, compare your result with an independent distance measurement rather than asking AI whether it looks plausible.

## Example 2: Trace a State Transition

In [Workshop 3](ws3.md), the `FORWARD` case in [main_ws3_p2.cpp](../solutions/main_ws3_p2.cpp) selects its next state using:

```cpp
if (us_distance_cm < 4.5f)
    robot_state = RobotState::EMERGENCY;
// emergency takes priority when both conditions are true
else if (motor_M3.getRotation() > 1.49f)
    robot_state = RobotState::BACKWARD;
```

For this paper trace, assume the program is executing that case and the stored distance and returned rotation have the values below. Predict the selected state before opening the table. Do not create these conditions by moving a hand into a mechanism.

<details>
<summary>Check the Selected States</summary>

| Distance (cm) | Rotation (turns) | State after this selection |
| ------------- | ---------------- | -------------------------- |
| 4.0           | 1.50             | `EMERGENCY`                |
| 5.0           | 1.50             | `BACKWARD`                 |
| 5.0           | 1.00             | `FORWARD`                  |

In the first row, the first condition is true, so the `else if` is skipped. In the last row, neither assignment executes and the state remains `FORWARD`. Assigning a state does not jump into another `case` of the same `switch`; inspect the surrounding loop to see when that state's code will execute.

</details>

**Ask for feedback:**

> Here is my trace of the `FORWARD` case in `docs/solutions/main_ws3_p2.cpp`: [my trace]. Check the order of evaluation and whether I confused selecting a state with executing its actions. Explain one mistake at a time. Do not edit the code or change its thresholds.

**Try without the answer:** trace another pair of values and explain why the branch is selected. This checks program flow; it does not establish physical stopping behaviour or validate the whole controller. Follow the existing [Hardware Precautions](../../README.md#hardware) for any practical work.

## Example 3: Check Units Before Asking for a Filter Fix

In [Workshop 5](ws5.md), [main_comp_filter.cpp](../solutions/main_comp_filter.cpp) sends a time interval in microseconds, gyro X in radians per second, acceleration Y/Z in metres per second squared and roll in radians. The [Python Stream Reader](../templates/python/SerialStream.py) converts the intervals into time in seconds and removes the time field from the measurement array: gyro X is therefore column `0` of `data["values"]`. In the transmitted sequence of floats, the interval has index `0` and gyro X has index `1`; these are zero-based indices, not byte positions. The [Python Evaluation Template](../templates/python/serial_stream_comp_filter.py) uses that layout and converts the plotted roll estimates to degrees.

For a simple arithmetic check, a constant rate of `1 rad/s` over `20,000 µs` gives an angle increment of `0.02 rad`: first convert the interval to `0.02 s`. This is an illustrative calculation, not a measured sample interval or a replacement for the filter algorithm.

If two curves disagree, ask for a units check before requesting new filter code:

> I am comparing my Workshop 5 roll estimate with the recorded reference. Here is my calculation, the units of each input and the relevant producer and evaluation code: [my attempt and excerpts]. Check units, indexing and initial conditions. Suggest one check before proposing a code change. Do not implement the workshop filters for me or invent measurement results.

**Try without the answer:** derive the units of your gyro update and check where conversion to degrees occurs. Compare the actual timestamps and data columns with the stream reader. Keep the input data and initial conditions fixed when comparing algorithms. The [Sensor Fusion Tutorial](imu.md#sensor-fusion) provides the course context.

## Use the Same Approach Across All Five Workshops

These suggestions complement the existing optional self-checks:

| Workshop                                   | Focus for your explanation                                                                                                                                                                                                                                                                                                                                           |
| ------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [1: IR Calibration](ws1.md)                | Trace a reading from normalized input to voltage and calibrated distance.                                                                                                                                                                                                                                                                                            |
| [2: Servos and State Machines](ws2.md)     | Sketch the states of your program and predict a transition for a given sensor observation. Compare servo commands with the calibration in [Servo](servo.md).                                                                                                                                                                                                         |
| [3: DC Motor Control](ws3.md)              | Distinguish state selection, motor commands and observed motion.                                                                                                                                                                                                                                                                                                     |
| [4: Kinematics and Line Following](ws4.md) | Trace the units through one calculation. In [main_dd_kinematic_calib.cpp](../solutions/main_dd_kinematic_calib.cpp), wheel angles are divided by `2.0f * M_PIf` before `setRotationRelative()`. Check the conversion from radians to turns against the [DCMotor Implementation](../../lib/DCMotor/DCMotor.cpp), where encoder counts are divided by counts per turn. |
| [5: Sensor Fusion](ws5.md)                 | Explain a plotted signal's source, unit and processing before comparing estimates.                                                                                                                                                                                                                                                                                   |

## When You Need Debugging Help

Start with the first relevant error or unexpected observation. Provide a small code section, your expectation, what happened and what you already checked. Ask for one hypothesis and a way to test it. See [Asking for Debugging Help](tips.md#asking-for-debugging-help).

Check proposed APIs against the supplied headers and installed library version. Read every changed line; a summary may omit edits. A successful build checks compilation and linking, not wiring, calibration or physical behaviour. AI approval, generated test claims and agreement between chatbots do not replace checking actual sources, outputs and measurements.

If a proposal conflicts with the course material or observed behaviour, preserve your working version and describe the discrepancy to a teaching assistant. Review firmware changes before upload; leave pins, button handling, timing and hardware parameters outside an unrelated debugging change.

## Check What You Learned

Close the generated answer and explain the relevant code, predict another case and make a small related change yourself when appropriate. Use documentation as needed. If you cannot do a step yet, name the missing concept and ask for focused help. A classmate can listen and ask you to trace the next step.

Keep a short note of your hypothesis, check, result and remaining question; see [Keeping Useful Notes](tips.md#keeping-useful-notes). If you used AI, noting the tool, purpose, adopted parts and your checks can help you explain your process. Follow the format required for assessed work.

Share only material you are permitted to send to the chosen service. Remove credentials and personal or confidential information from prompts and logs. An assistant inside VS Code may still send context to an external service.

## Basis of This Guide

The routine draws on [PRIMM](https://suesentance.net/2019/05/06/new-paper-primm-and-sociocultural-theory/) and research on [Retrieval Practice and Spaced Study](https://doi.org/10.1177/1529100612453266). Studies of [Novice Programmers Using AI](https://doi.org/10.1145/3632620.3671116) and [AI-Supported Programming Exercises](https://doi.org/10.1016/j.caeai.2025.100537) motivate checking understanding separately from task completion. These sources do not establish the effectiveness of this particular guide.

Prepared with AI assistance. The [Teaching Rationale](../notes/ai_learning/README.md) summarises the sources and limitations behind these recommendations. Reviewed against the linked repository examples on 21 October 2026.
