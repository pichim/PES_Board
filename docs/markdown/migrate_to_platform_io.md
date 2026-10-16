# PlatformIO-only Documentation: Work List

Updated **2026-10-16**. Scope: README.md and the original baseline Markdown files. Only unresolved work is listed below, followed by the evidence collected during review.

Target: one PlatformIO setup/build/upload/clean workflow using the existing Mbed framework and project configuration, with Microsoft's Serial Monitor extension for serial output. Setup, build and workshop instructions have been consolidated. The standalone Linux/Windows build guides have been removed.

The user checked all 28 external URLs in the manual review list and confirmed that only the WS5 text link was wrong. WS5 now uses its confirmed QR destination; the teaching-plan quiz links match the corresponding workshops, with no `Link TBA` placeholders remaining. These checks are resolved.

## Remaining Hardware Description Check

- [imu.md](imu.md): the gimbal CAD link points to the existing `docs/cad/WS5` files. Verify that the stated Modelcraft RS2 MG/BB servo model matches these parts before treating the mechanical fit as confirmed. Opening the link does not verify that fit.

## Remaining Validation for the Setup and Build Guides

The canonical workflow is in [build_mbed_platformio.md](build_mbed_platformio.md). [course_setup.md](course_setup.md) covers prerequisites, extension installation, forking and cloning, and links to that workflow.

- **Fresh installation:** test installation and the first build on each course operating system. On Ubuntu 24.04, validate the documented compatibility repair against a fresh package installation; the successful Linux build used an existing Python environment and an already-patched Mbed package. On Windows, validate the normal workflow without this repair.
- **Physical board and VS Code:** test PlatformIO Build, Upload and Clean, plus Microsoft's Serial Monitor extension, in the current UI with the Nucleo-F446RE. Confirm upload-drive availability, detection and write access for `upload_protocol = mbed`, the default program's LED/button behaviour, and 115200-baud output using code that prints. The default program has no startup message. The ST-LINK mass-storage interface must be enabled for the configured upload method.
- **Reproducibility:** decide whether to pin the tested platform/dependency versions before course deployment. `platform = ststm32` and the compatibility procedure's setuptools requirement remain unpinned. Record the versions from fresh-install tests as well as the existing-install build below.
- **Student checkout:** before distributing the course instructions, ensure the branch students clone contains the final documentation, `platformio.ini`, `mbed_app.json` and both VS Code extension recommendations. The review is on `dev`; the locally recorded `origin/HEAD` points to `main`. Validate the published default branch or document the course branch when publishing.

## Review Evidence and Limits

- The user confirmed that the Mbed Python compatibility issue occurred only on Ubuntu 24.04, not on Windows in the course setup. The repair is documented for Ubuntu 24.04; a Windows version of that Ubuntu-specific dependency patch is not required.

- The unchanged default program previously built successfully in an isolated directory on Linux with PlatformIO Core 6.2.0, official ST STM32 19.0.0, Mbed OS 6.17.0 (`framework-mbed` 6.61700.231105), GCC 9.2.1, Python 3.12.3 and setuptools 83.0.0. The installed dependency already used `importlib.reload`; this was not a fresh-install or hardware test.
- The final review checks both the staged changes and the current working files. Local Markdown links, images, heading anchors and quiz destinations are checked separately from external-link availability, which the user verified.
- The documented Bash repair now uses Python's syntax tree to identify the import, validates the target before running pip, checks that it runs in the expected Python environment, preserves line endings, and preserves any existing backup. Fourteen isolated cases passed, including misleading comments/docstrings, an already-patched file, CRLF, an existing backup, missing/unexpected files, pip failure and the wrong Python environment. Pip was mocked in these cases; no global packages were changed. Bash and embedded Python syntax checks passed. A complete fresh-install build after this revised repair remains part of the installation check above.
- The SerialStream wiring text now agrees with `PESBoardPinMap.h`: adapter TX goes to MCU RX (`PC_5`), adapter RX to MCU TX (`PB_10`), plus common ground. LED timing and the LED's series resistor/polarity are documented consistently with the default source. The right Arduino-header image is labeled correctly. The retained GitHub screenshot is explicitly identified as a generic illustration from another repository.

- Installation troubleshooting was checked against PlatformIO's official IDE settings, Python installation, Core troubleshooting, uninstall and directory documentation, Python's environment-variable documentation, and the locally cached PlatformIO Core Installer 1.2.2 source. The guide documents conditional Windows interpreter/environment conflicts and recovery from a persistently broken installation on all operating systems. A global Python `PATH` entry alone is not a proven fault, and deleting the complete Core directory after every interruption is not a proven requirement. The installer source recreates `penv` during virtual-environment creation; full Core removal is a broader reset. These instructions have not been tested by deliberately interrupting a fresh installation on each OS.

- Microsoft's Serial Monitor (`ms-vscode.vscode-serial-monitor`) is now the recommended monitor, per the user's preference. Its Marketplace instructions and the locally installed 0.13.1 extension confirm the panel and Start/Stop Monitoring controls. Students set 115200 baud in that extension explicitly; PlatformIO's `monitor_speed` does not configure it. The physical-board check above remains required.

Do not treat the migration as fully validated for student distribution until the remaining installation, publication and physical-board checks are complete. Binary UART streams use their own settings, as described in [SerialStream](serial_stream.md).
