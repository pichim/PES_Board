# PlatformIO-only Documentation: Work List

Updated **2026-10-16**. Scope: README.md and the 27 baseline files in `docs/markdown/`. Only files and passages needing work are listed. Source line numbers refer to commit `170cc5c`; source guides have not yet been changed.

Target: one PlatformIO setup/build/upload/monitor workflow. Remove Studio, standalone build guides and alternative compiler/terminal workflows. The existing Mbed framework and project configuration remain part of the PlatformIO setup.

**Link priority:** fix every confirmed broken or incorrect link. For each UNVERIFIED destination, check it in a browser and replace/remove it if unavailable or wrong; an automated fetch failure alone does not establish that it is dead. When deleting a guide or heading, update incoming links in the same change.

## Work by File

### [README.md](../../README.md)

**Links**

| Source line(s) | Required work                                                                                                                                                                                       |
| -------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 25             | **UNVERIFIED:** `https://www.printables.com/de/model/522220-universal-gear-rack-collection-different-modules`. Check availability in a browser; replace/remove if unavailable or incorrect.         |
| 26             | **UNVERIFIED:** `https://www.printables.com/de/model/516177-universal-spur-gears-collection-module-15`. Check availability in a browser; replace/remove if unavailable or incorrect.                |
| 92, 122, 193   | **REPLACE:** obsolete `https://os.mbed.com/platforms/ST-Nucleo-F446RE/` definition and uses. Point to `docs/markdown/mbed_nucleo_f446re_pin_map.md` and update labels.                              |
| 102            | **UNVERIFIED:** `https://github.com/szar99`. Check availability in a browser; replace/remove if unavailable or incorrect.                                                                           |
| 140            | **BROKEN LOCAL:** `docs/markdown/tips.md#programming` (Programming). Replace with `docs/markdown/tips.md#programming-tips`.                                                                         |
| 149            | **BROKEN LOCAL:** `#build-mbed-os-projects-with-vs-code` (Build Mbed OS projects with VS Code). Replace with `#build-mbed-os-projects-with-vs-code-with-platformio` (or the final renamed heading). |
| 286            | **BROKEN LOCAL:** `docs/markdown/tips.md#programming` (Programming). Replace with `docs/markdown/tips.md#programming-tips`.                                                                         |
| 349            | **UNVERIFIED:** `https://boxtec.ch`. Check availability in a browser; replace/remove if unavailable or incorrect.                                                                                   |
| 351            | **UNVERIFIED:** `http://farnell.ch`. Check availability in a browser; replace/remove if unavailable or incorrect.                                                                                   |
| 355            | **UNVERIFIED:** `https://www.distrelec.ch`. Check availability in a browser; replace/remove if unavailable or incorrect.                                                                            |

**Migration changes**

| Source line(s)   | Required work                                                                                                                                                    |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 36–39            | Remove the completed PlatformIO TODO and obsolete GCC/armclang, Studio-compatibility and PuTTY TODO comments.                                                    |
| 47–48            | Remove the historical checklist entries for the standalone Linux/Windows build guides.                                                                           |
| 119, 164         | Retarget `course_setup.md#arm-mbed` to the final PlatformIO setup heading; update both link labels.                                                              |
| 151–152, 323–328 | Remove the standalone Windows/Linux build links and the entire “without PlatformIO” section. Update the contents list to expose only the PlatformIO build guide. |
| 189              | Replace the old Arm Mbed website introduction with the local pin map and ST board resources.                                                                     |
| 340              | Remove the OnlineGDB compiler link; PlatformIO is the only documented compilation workflow.                                                                      |

### [1d_mahony.md](1d_mahony.md)

**Links**

| Source line(s) | Required work                                                                                                                                              |
| -------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2–4            | **REMOVE:** unused external reference definitions `[0]`, `[1]`, `[2]`. Removing them also removes the unverified visualization/video links from this file. |

### [build_mbed_linux.md](build_mbed_linux.md)

**Migration changes**

| Source line(s) | Required work                                                                                                                                                                                                                                                                                                            |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 1–169          | Remove this entire standalone build guide, including commented-out instructions, download/share URLs, Conda, manual GCC, Node/npm, Mbed CLI/generator, CMake/Makefile extensions, WSL, symlink and separate monitor workflows. Update all incoming links; no individual URL checks are needed for content being deleted. |

### [build_mbed_platformio.md](build_mbed_platformio.md)

**Migration changes**

| Source line(s) | Required work                                                                                                                                                                                                                                    |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 11             | Resolve the C/C++ Extension Pack recommendation conflict with `.vscode/extensions.json:8`, where that pack is unwanted. Distinguish the pack from the individual C/C++ extension.                                                                |
| 13             | Correct the claim that PlatformIO automatically installs Linux udev rules. Use the [official USB/serial permission instructions](https://docs.platformio.org/en/latest/core/installation/udev-rules.html).                                       |
| 14             | Remove PuTTY as an alternative terminal; use PlatformIO Monitor.                                                                                                                                                                                 |
| 16, 31, 35     | Document the Python compatibility prerequisites where required; verify a fresh first build before claiming no extra command-line steps are needed. Integrate the [recorded compatibility procedure](#platformiombed-python-compatibility-setup). |
| 18, 25, 27     | Remove the Studio transition heading and comparison with `.mbed`/Studio profiles. Consolidate the guide into one PlatformIO setup path.                                                                                                          |
| 35             | Correct “all configuration lives in platformio.ini”: explain the role of `mbed_app.json` alongside the PlatformIO environment configuration.                                                                                                     |

### [build_mbed_windows.md](build_mbed_windows.md)

**Migration changes**

| Source line(s) | Required work                                                                                                                                                                                                                                                                                               |
| -------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1–166          | Remove this entire standalone build guide, including commented-out instructions, download/share URLs, Conda, manual GCC, Node/npm, Mbed CLI/generator, CMake/Makefile extensions, WSL, symlink and separate monitor workflows. Remove its link to the Linux standalone guide and update all incoming links. |

### [changes_for_flipped_classroom.md](changes_for_flipped_classroom.md)

**Links**

| Source line(s)     | Required work                                                                                                                                                                                                              |
| ------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 13                 | **UNVERIFIED:** `https://www.qrcode-generator.ch/`. Browser returned a challenge page; verify interactively if this optional planning resource is retained.                                                                |
| 234                | **REVIEW / UNVERIFIED:** `https://forms.cloud.microsoft/e/knqQAypewF` differs from WS1’s `BFmzMtMYfv` short URL (`ws1.md:3,38`). Verify both redirects and the QR target; synchronize the correct quiz link in both files. |
| 391, 536, 690, 845 | **MISSING DESTINATIONS:** WS2–WS5 announcement templates still say `Link TBA`. Fill each quiz URL and reconcile its QR target. These are placeholders, not confirmed HTTP failures.                                        |

**Migration changes**

| Source line(s)                    | Required work                                                                                                                   |
| --------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| 22–23, 52                         | Remove references to the standalone Linux/Windows build guides; use the PlatformIO guide in the source mappings.                |
| 97, 103, 235                      | Replace Studio installation/opening requirements with the PlatformIO setup requirement.                                         |
| 98, 104, 236                      | Remove Mbed account requirements.                                                                                               |
| 116, 131, 155                     | Rewrite the Studio-specific quiz distractors for the PlatformIO-only materials.                                                 |
| 190, 214, 360, 375, 511, 664, 819 | Replace Studio import/target/build setup blocks and checkpoints with opening the configured PlatformIO project and building it. |
| 229                               | Change the `course_setup.md` reading description from Studio basics to PlatformIO setup.                                        |
| 239, 274, 393, 538, 692, 847      | Update every workshop prerequisite/announcement that requires Studio; remove the Mbed account prerequisite at 274 too.          |

### [color_sensor.md](color_sensor.md)

**Links**

| Source line(s) | Required work                                                                                                                                             |
| -------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2, 40          | **UNVERIFIED:** `https://www.mouser.com/catalog/specsheets/TCS3200-E11.pdf`. Check availability in a browser; replace/remove if unavailable or incorrect. |
| 3, 41          | **UNVERIFIED:** `https://wiki.dfrobot.com/sen0101`. Check availability in a browser; replace/remove if unavailable or incorrect.                          |

### [course_setup.md](course_setup.md)

**Links**

| Source line(s) | Required work                                                                                                                    |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| 16             | **REMOVE:** `https://www.putty.org/` returned unrelated content. Remove the link and PuTTY instructions; use PlatformIO Monitor. |

**Migration changes**

| Source line(s)                               | Required work                                                                                                                                                                                                                                                                                                                                                                                     |
| -------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 7, 10                                        | Remove the Mbed account prerequisite and registration link.                                                                                                                                                                                                                                                                                                                                       |
| 15–16                                        | Remove Studio and PuTTY from the software list; make PlatformIO IDE/Monitor the documented environment.                                                                                                                                                                                                                                                                                           |
| 20                                           | Provide Git installation guidance for each supported operating system; the present URL covers Windows only.                                                                                                                                                                                                                                                                                       |
| 38–49, 55–136                                | Remove the Arm Mbed/Studio onboarding, import dialogs, target-selection steps, build profiles, HAMMER/PLAY instructions and build-folder cleanup advice. Replace with the single PlatformIO setup workflow.                                                                                                                                                                                       |
| 47–48, 61–62, 76–77, 84–85, 117–118, 128–129 | Remove/replace all six Studio screenshot references and their captions.                                                                                                                                                                                                                                                                                                                           |
| 138–160                                      | Consolidate extension prerequisites with the PlatformIO guide. Remove the separate Serial Monitor extension workflow; reconcile the extension-pack recommendation with `.vscode/extensions.json`.                                                                                                                                                                                                 |
| 167, 191, 197                                | Make project-ready/first-build claims conditional on documented compatibility prerequisites and fresh-install verification.                                                                                                                                                                                                                                                                       |
| 187–188                                      | Correct the project-open screenshot’s “Flashing the Board” label/caption.                                                                                                                                                                                                                                                                                                                         |
| 196                                          | Correct the build-profile explanation: distinguish generic PlatformIO `build_type` from [Mbed framework profiles](https://docs.platformio.org/en/latest/frameworks/mbed.html#build-profiles), selected through `MBED_BUILD_PROFILE_RELEASE` / `MBED_BUILD_PROFILE_DEBUG`; the installed adapter defaults to `develop`. Avoid arbitrary optimization advice without validating the compiler flags. |
| 201–202, 209–210                             | Check build/upload screenshots and captions against the course’s actual PlatformIO UI; replace outdated images.                                                                                                                                                                                                                                                                                   |
| 213–228                                      | Replace blanket global-Python PATH removal and deleting all of `.platformio` with targeted troubleshooting, including the compatibility procedure where applicable.                                                                                                                                                                                                                               |
| 234–236, 250–252                             | Replace the manual drag-and-drop workflow with PlatformIO Upload. Document drive accessibility/detection because the configured `upload_protocol = mbed` uses the upload disk.                                                                                                                                                                                                                    |
| 238–248                                      | Replace Studio workspace and `BUILD/.../ARMC6/...bin` paths with the PlatformIO project location and expected `.pio/build/nucleo_f446re/firmware.bin` artifact.                                                                                                                                                                                                                                   |

### [dc_motor.md](dc_motor.md)

**Links**

| Source line(s) | Required work                                                                                                  |
| -------------- | -------------------------------------------------------------------------------------------------------------- |
| 203            | **BROKEN LOCAL:** `#motor-m1-open-loop` (Motor M1). Replace with `#motor-m1-open-loop-direct-voltage-control`. |

### [imu.md](imu.md)

**Links**

| Source line(s) | Required work                                                                                                                                                                                                                                             |
| -------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 3, 164         | **UNVERIFIED:** `https://compsci290-s2016.github.io/CoursePage/Materials/EulerAnglesViz/`. Check that the visualization renders in a browser; replace/remove if it does not.                                                                              |
| 4, 116         | **UNVERIFIED:** `https://www.youtube.com/watch?v=whSw42XddsU`. Check availability in a browser; replace/remove if unavailable or incorrect.                                                                                                               |
| 5, 43          | **UNVERIFIED:** `https://www.youtube.com/watch?v=wCk-FWCbskw`. Check availability in a browser; replace/remove if unavailable or incorrect.                                                                                                               |
| 6, 49          | **UNVERIFIED:** `https://www.youtube.com/watch?v=uDJ_SURe3c0`. Check availability in a browser; replace/remove if unavailable or incorrect.                                                                                                               |
| 7, 56          | **REVIEW TARGET:** `[5]` is labeled as a magnetometer video in MEMS/Hall-effect context, while the fetched YouTube title is about fluxgate magnetometers. Check whether it teaches the intended sensor principle; reachable does not establish relevance. |
| 254            | **BROKEN LOCAL:** `../cad/gimbal` (here). Replace with `../cad/WS5` (directory containing the gimbal STL/STP files; verify the servo/model description separately).                                                                                       |

### [ir_sensor.md](ir_sensor.md)

**Links**

| Source line(s) | Required work                                                                                                                                            |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 6, 66          | **REPLACE:** obsolete `https://os.mbed.com/platforms/ST-Nucleo-F446RE/` definition and uses. Point to `mbed_nucleo_f446re_pin_map.md` and update labels. |

### [line_follower.md](line_follower.md)

**Links**

| Source line(s) | Required work                                                                                                                                                                  |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 3, 36          | **UNVERIFIED:** `https://learn.sparkfun.com/tutorials/sparkfun-line-follower-array-hookup-guide`. Check availability in a browser; replace/remove if unavailable or incorrect. |
| 4, 48          | **WRONG TOPIC:** the I²C explanation at line 48 uses `[2]`, whose URL is `serial-peripheral-interface-spi/all`. Replace the SPI URL with a verified I²C resource.              |
| 5, 55          | **REPLACE:** obsolete `https://os.mbed.com/platforms/ST-Nucleo-F446RE/` definition and uses. Point to `mbed_nucleo_f446re_pin_map.md` and update labels.                       |

### [sd_card_logger.md](sd_card_logger.md)

**Migration changes**

| Source line(s) | Required work                                                                                                                                                                                                 |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 7              | Replace the GCC-versus-Arm Compiler 6/armclang/Studio warning with the supported PlatformIO setup. Do not retain an alternative compiler workflow or claim compatibility testing that has not been performed. |

### [servo.md](servo.md)

**Links**

| Source line(s) | Required work                                                                                                                                                                                                                                                 |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2, 30          | **REVIEW TARGET:** link labeled `Futaba S3001` points to `?cat=21&tit=Servo%20SBus`, a category URL whose title does not identify S3001. Verify and choose a model-specific manufacturer/datasheet destination. Availability and relevance remain unverified. |
| 3, 31          | **UNVERIFIED:** `https://www.conrad.ch/de/p/reely-standard-servo-cys-s0090-analog-servo-getriebe-material-metall-stecksystem-jr-2203091.html?refresh=true#productHighlights`. Check availability in a browser; replace/remove if unavailable or incorrect.    |
| 5              | **CLEAN UP UNUSED DEFINITION:** `[3]` has no reference uses in this file. Remove this obsolete Mbed URL definition; no Nucleo-page replacement is needed here.                                                                                                |

### [stepper_motor.md](stepper_motor.md)

**Links**

| Source line(s) | Required work                                                                                                                                                                                                   |
| -------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 4–5, 63        | **WRONG REFERENCE:** change `[A4988 Stepper Motor Driver Carrier][3]` at line 63 to `[2]` (`https://www.pololu.com/product/1182`, defined at line 4), then remove the obsolete Mbed definition `[3]` at line 5. |
| 6, 63          | **UNVERIFIED:** `https://www.youtube.com/watch?v=89BHS9hfSUk&t=1s`. Check availability in a browser; replace/remove if unavailable or incorrect.                                                                |

### [ultrasonic_sensor.md](ultrasonic_sensor.md)

**Links**

| Source line(s) | Required work                                                                                                                                                                                                                                             |
| -------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2, 30          | **UNVERIFIED:** `https://www.digikey.ch/de/products/detail/seeed-technology-co-ltd/101020010/5482600?s=N4IgTCBcDaIOICUDyA1AogAgKoBkAqCAggMpIByAkgMIZFlxoIYpgB0ADCALoC%2BQA`. Check availability in a browser; replace/remove if unavailable or incorrect. |

### [ws1.md](ws1.md)

**Links**

| Source line(s) | Required work                                                                                                                                                                                            |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2, 71          | **REPLACE:** obsolete `https://os.mbed.com/platforms/ST-Nucleo-F446RE/` definition and uses. Point to `mbed_nucleo_f446re_pin_map.md` and update labels.                                                 |
| 3, 38          | **UNVERIFIED:** `https://forms.cloud.microsoft/e/BFmzMtMYfv`. Check availability, intended workshop and Studio wording; reconcile with the [decoded QR target](#quiz-links-and-qr-targets-to-reconcile). |

**Migration changes**

| Source line(s) | Required work                                                                                                                                                           |
| -------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 7, 32, 95      | Replace Studio in the workshop introduction, setup-reading description and summary with PlatformIO.                                                                     |
| 72–74, 76      | Rewrite console/port detection, active-program selection, PLAY flashing and progress instructions around PlatformIO Upload and Monitor; remove the Studio alternatives. |

### [ws2.md](ws2.md)

**Links**

| Source line(s) | Required work                                                                                                                                                                                            |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2, 114         | **REPLACE:** obsolete `https://os.mbed.com/platforms/ST-Nucleo-F446RE/` definition and uses. Point to `mbed_nucleo_f446re_pin_map.md` and update labels.                                                 |
| 3, 36          | **UNVERIFIED:** `https://forms.cloud.microsoft/e/csBg2wWZa5`. Check availability, intended workshop and Studio wording; reconcile with the [decoded QR target](#quiz-links-and-qr-targets-to-reconcile). |

**Migration changes**

| Source line(s) | Required work                                                                 |
| -------------- | ----------------------------------------------------------------------------- |
| 295            | Replace the Studio PLAY build/flash instruction with PlatformIO Build/Upload. |

### [ws3.md](ws3.md)

**Links**

| Source line(s) | Required work                                                                                                                                                                                            |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2, 108         | **REPLACE:** obsolete `https://os.mbed.com/platforms/ST-Nucleo-F446RE/` definition and uses. Point to `mbed_nucleo_f446re_pin_map.md` and update labels.                                                 |
| 3, 35          | **UNVERIFIED:** `https://forms.cloud.microsoft/e/U3v2D8G4Em`. Check availability, intended workshop and Studio wording; reconcile with the [decoded QR target](#quiz-links-and-qr-targets-to-reconcile). |
| 143            | **BROKEN LOCAL:** `dc_motor.md#motor-m3` (Tutorial DC Motor). Replace with `dc_motor.md#motor-m3-closed-loop-position-control`.                                                                          |

**Migration changes**

| Source line(s) | Required work                                                      |
| -------------- | ------------------------------------------------------------------ |
| 305            | Replace the Studio PLAY upload instruction with PlatformIO Upload. |

### [ws4.md](ws4.md)

**Links**

| Source line(s) | Required work                                                                                                                                                                                                                                                                                                            |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 2              | **CLEAN UP UNUSED DEFINITION:** `[0]` defines the old Mbed Nucleo URL but has no reference uses in WS4. Remove rather than adding an unnecessary replacement.                                                                                                                                                            |
| 3, 34          | **REVIEW / UNVERIFIED:** `https://forms.cloud.microsoft/e/rkadzrxAGC` is used for both WS4 and WS5, while their QR images encode different full URLs. Verify availability, redirects and intended workshop; synchronize the correct text link and QR image. See [quiz targets](#quiz-links-and-qr-targets-to-reconcile). |
| 30             | **BROKEN LOCAL:** `../README.md` (README.md). Replace with `../../README.md`.                                                                                                                                                                                                                                            |
| 32             | **BROKEN LOCAL:** `line_follower_array.md` (Line Follower). Replace with `line_follower.md`.                                                                                                                                                                                                                             |

### [ws5.md](ws5.md)

**Links**

| Source line(s) | Required work                                                                                                                                                                                                                                                                                                            |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 2              | **CLEAN UP UNUSED DEFINITION:** `[0]` defines the old Mbed Nucleo URL but has no reference uses in WS5. Remove.                                                                                                                                                                                                          |
| 3, 36          | **REVIEW / UNVERIFIED:** `https://forms.cloud.microsoft/e/rkadzrxAGC` is used for both WS4 and WS5, while their QR images encode different full URLs. Verify availability, redirects and intended workshop; synchronize the correct text link and QR image. See [quiz targets](#quiz-links-and-qr-targets-to-reconcile). |
| 32             | **BROKEN LOCAL:** `../README.md` (README.md). Replace with `../../README.md`.                                                                                                                                                                                                                                            |

## Quiz Links and QR Targets to Reconcile

Resolve the different WS1 short URLs (`BFmzMtMYfv` in `ws1.md`, `knqQAypewF` in the teaching plan), the shared WS4/WS5 short URL (`rkadzrxAGC`), and all `Link TBA` entries. Confirm intended workshop, access and content, then synchronize text links, announcements and QR images.

The five images decode to distinct full URLs below. Equivalence to their short links and form availability are still unverified; a short URL may legitimately redirect to a different-looking full URL.

| Local QR image                     | Source occurrence(s)                                | Decoded address                                                                                                                             |
| ---------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| `docs/images/ws1_quiz_qr_code.png` | `ws1.md:41`                                         | `https://forms.cloud.microsoft/Pages/ResponsePage.aspx?id=nZ8aXR8gEEq5g0Uc9ly8Hj4DVXk-SFpLrn4htuIp731UNUxRNDMxNjRYMFlDWFpQVzZVNTc3NDZORy4u` |
| `docs/images/ws2_quiz_qr_code.png` | `ws2.md:39`; `changes_for_flipped_classroom.md:391` | `https://forms.cloud.microsoft/Pages/ResponsePage.aspx?id=nZ8aXR8gEEq5g0Uc9ly8Hj4DVXk-SFpLrn4htuIp731UQVhSQTVQTklUWUNDQzI3QTZSWjEzNkFEMy4u` |
| `docs/images/ws3_quiz_qr_code.png` | `ws3.md:38`; `changes_for_flipped_classroom.md:536` | `https://forms.cloud.microsoft/Pages/ResponsePage.aspx?id=nZ8aXR8gEEq5g0Uc9ly8Hj4DVXk-SFpLrn4htuIp731UOFUzVExBVjJXWlFFSzdSNTVHTzRDWDc4OC4u` |
| `docs/images/ws4_quiz_qr_code.png` | `ws4.md:37`                                         | `https://forms.cloud.microsoft/Pages/ResponsePage.aspx?id=nZ8aXR8gEEq5g0Uc9ly8Hj4DVXk-SFpLrn4htuIp731UQUVNT1VJMUtLQTQwTlpRSDVXRFc3MFNSNi4u` |
| `docs/images/ws5_quiz_qr_code.png` | `ws5.md:39`; `changes_for_flipped_classroom.md:845` | `https://forms.cloud.microsoft/Pages/ResponsePage.aspx?id=nZ8aXR8gEEq5g0Uc9ly8Hj4DVXk-SFpLrn4htuIp731UNTJWR05QWVc5N1cwUDJZUUo3M1NKTk9GTS4u` |

## PlatformIO/Mbed Python Compatibility Setup

Integrate this user-provided procedure into the PlatformIO setup/troubleshooting guide where required, after testing it against a fresh installation. Adapt and validate the paths/shell for Windows. The procedure assumes the default `~/.platformio` location and an already-installed Python environment and `framework-mbed` package.

```bash
(
set -e

# Install setuptools into PlatformIO's Python environment.
~/.platformio/penv/bin/python -m pip install setuptools

# Patch Mbed's obsolete Python import.
~/.platformio/penv/bin/python - <<'PY'
from pathlib import Path
import shutil

path = Path.home() / (
    ".platformio/packages/framework-mbed/"
    "platformio/package_deps/py3/past/builtins/misc.py"
)

if not path.is_file():
    raise SystemExit(f"ERROR: Mbed dependency file not found: {path}")

text = path.read_text()

if "from importlib import reload" in text:
    print("Import already patched.")
elif "from imp import reload" in text:
    backup = path.with_suffix(".py.bak")
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(
        text.replace("from imp import reload", "from importlib import reload")
    )
    print("Import patched; backup saved as misc.py.bak.")
else:
    raise SystemExit("ERROR: Unexpected file contents; no changes made.")
PY

echo "PlatformIO compatibility setup completed."
)
```

Before presenting it as a supported repair procedure:

- Verify the active import and run a real firmware build. The substring check can report “already patched” when the new import exists only in a comment.
- Explain that pip runs before the file checks, setuptools is unpinned, an existing backup is preserved, and reinstalling/updating the shared framework package may overwrite the patch.
- Validate the complete setup on each supported OS. Only syntax and temporary-file patch behavior have been checked; pip, the real package patch and a fresh firmware build were not executed during this documentation review.

## Completion Checks

- Verify every retained external link and quiz destination; resolve all remaining UNVERIFIED/REVIEW items.
- Recheck local paths, heading anchors and incoming references after deleting the standalone guides and Studio sections.
- Scan all 28 source files, including HTML comments and quiz distractors, for remaining Studio and standalone-tool instructions.
- Test fresh installation, compatibility setup, Build, Upload, Monitor and Clean on the supported operating systems and actual Nucleo hardware. Record the tested version set; `platform = ststm32` is currently unpinned.
- Validate screenshots and captions against the course UI. Confirm upload-disk detection for `upload_protocol = mbed` and the 115200-baud console; binary UART streams may use a different baud rate.
