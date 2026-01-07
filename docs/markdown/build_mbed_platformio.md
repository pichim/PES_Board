# Build Mbed OS Projects with PlatformIO in VS Code

## Prerequisites

Install these before opening the project in VS Code:

- Git
- Visual Studio Code
- VS Code extensions:
  - PlatformIO IDE
  - C/C++ Extension Pack (Microsoft)
- Windows only: ST-LINK USB driver (STSW-LINK009)
- Linux only: add your user to the `dialout` group (log out/in after) so PlatformIO can access the USB serial device; PlatformIO installs the udev rules automatically.
- Optional on Windows: PuTTY (for an alternative serial terminal).

PlatformIO automatically downloads the ARM GCC toolchain and the required Mbed framework the first time you build.

## Using PlatformIO after having used Mbed Studio

- Open the repository folder in VS Code. PlatformIO detects `platformio.ini` automatically.
- Build: use the **PlatformIO: Build** action (check the blue status bar or the Command Palette).
- Flash: use **PlatformIO: Upload**. Make sure the ST-LINK driver is installed on Windows.
- Monitor: use **PlatformIO: Monitor**. Default baud is 115200; adjust in the status bar or Command Palette if needed.
- Clean: use **PlatformIO: Clean**.
- IntelliSense comes from the PlatformIO extension; no `.mbed` file or Mbed Studio profiles are needed.

## Only using PlatformIO

1. Install the prerequisites above.
2. Clone your repository and open the folder in VS Code.
3. When PlatformIO finishes indexing, run **PlatformIO: Build**.
4. Connect the Nucleo-F446RE via USB and run **PlatformIO: Upload**.
5. Start **PlatformIO: Monitor** to view serial output (default 115200 baud).

All configuration lives in `platformio.ini`; board `nucleo_f446re` is already set up. No additional command-line steps are required when working from VS Code.
