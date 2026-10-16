# Build Mbed OS Projects with PlatformIO in VS Code

Use this guide for the PES Board repository and the Nucleo-F446RE. The project uses PlatformIO with the Mbed OS 6 framework. Complete the [course setup](course_setup.md) first.

## Open the Project

1. Open VS Code and select **File > Open Folder**.
2. Select the repository folder you cloned during [course setup](course_setup.md#clone-your-fork). The folder must contain `platformio.ini`, `mbed_app.json`, `src/`, `include/` and `lib/`.
3. If VS Code asks about workspace trust, review the repository and trust it to enable its development tools.
4. Allow PlatformIO IDE to initialise. Use the `nucleo_f446re` environment for the following tasks.
5. Open the existing [src/main.cpp](../../src/main.cpp). Use this default program for the first build; its structure is explained in [Main File Description](main_description.md).

## Project Configuration

The repository already contains the required configuration:

| File                                   | Purpose                                                                                                                      |
| -------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| [platformio.ini](../../platformio.ini) | STM32 platform, `nucleo_f446re` board, `mbed` framework, build flags, upload protocol and PlatformIO's own monitor settings. |
| [mbed_app.json](../../mbed_app.json)   | Mbed application settings, including SD support, `printf`, main-thread stack size and the firmware's serial baud rate.       |
| [src/main.cpp](../../src/main.cpp)     | Application entry point compiled for the workshops.                                                                          |
| `include/` and `lib/`                  | Project headers and local libraries.                                                                                         |

Keep the supplied build settings for the course. The firmware's serial baud rate is **115200**. Select that baud rate separately in Microsoft's Serial Monitor extension; `monitor_speed` in `platformio.ini` configures PlatformIO's own monitor and does not configure the Microsoft extension.

## Build

1. Save your files.
2. Select or press **PlatformIO: Build** in the VS Code status bar. Hover over a toolbar button to see its action name. You can also select **Terminal > Run Task > PlatformIO: Build**.
3. Read the build output in the terminal and wait for **SUCCESS**. If the build fails, address the first error before continuing; see [Troubleshooting](#troubleshooting).

A board connection is not required to compile. PlatformIO downloads the platform, toolchain and Mbed framework when required, so the first build needs internet access and can take longer. On Ubuntu 24.04, the Mbed Python dependencies may need the [compatibility repair](#mbed-python-compatibility-linux-only) below. This repair is not needed for the Windows course setup.

A successful build produces `.pio/build/nucleo_f446re/firmware.bin` inside the project folder.

## Upload

1. Follow the [hardware precautions in the README](../../README.md#hardware) before connecting or changing hardware.
2. Connect the Nucleo-F446RE's ST-LINK USB connector to your computer with a data-capable cable.
3. Check that the board's removable drive is available and writable. This project sets `upload_protocol = mbed`, which copies the firmware to that drive through the onboard ST-LINK interface. On Linux, mount the drive if your desktop has not done so.
4. Select or press **PlatformIO: Upload**, or select **Terminal > Run Task > PlatformIO: Upload**. Upload builds the program as needed and then flashes it.
5. Wait for **SUCCESS** in the terminal, then confirm that the board runs the expected program. With the default `main.cpp`, the onboard LED toggles rapidly and the blue USER button enables or disables the additional LED connected as described in [Workshop 1](ws1.md#part-1).

After changing code, save it and use **PlatformIO: Upload** again. A separate Build action is useful when you only want to check compilation.

### Optional Drag-and-Drop Upload

After a successful PlatformIO build, you can also copy `.pio/build/nucleo_f446re/firmware.bin` to the Nucleo's removable drive. Use the binary from the current successful build. This manual copy does not compile changes; **PlatformIO: Upload** remains the normal course workflow.

## Serial Monitor

Use the [Serial Monitor extension by Microsoft](https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor), installed during [course setup](course_setup.md#install-the-vs-code-extensions).

1. Connect the board, then select **Terminal > New Terminal** to open VS Code's bottom panel.
2. Select the **Serial Monitor** tab in that panel.
3. Select **Serial** mode and the Nucleo's ST-LINK virtual serial port from the port dropdown. If several devices are attached, identify the board by disconnecting and reconnecting it.
4. Set the baud rate to **115200**, with **8 data bits**, **no parity** and **1 stop bit**.
5. Select **Start Monitoring** to view the program's text output.

Select **Stop Monitoring** to release the port before opening another program that needs it. Use only one monitor on a given serial port at a time. If the port disconnects during an upload, select the board's port again and start monitoring after upload finishes.

The default `main.cpp` does not print a startup message or continuous serial output. An empty monitor by itself does not indicate a failed upload. Workshop code prints output when the corresponding `printf` statements execute. The default program toggles the onboard user LED in its main loop; its blue user button enables or disables the additional LED task.

Binary serial-stream examples use their own MATLAB/Python receivers and settings; follow the [Serial Stream tutorial](serial_stream.md) for those examples.

## Clean

Use **PlatformIO: Clean**, or **Terminal > Run Task > PlatformIO: Clean**, when you need to discard generated build output. Then use **PlatformIO: Build** or **PlatformIO: Upload** again. Clean does not remove the project's source files.

## Troubleshooting

### Windows Python and Environment Conflicts

A conflicting global Python installation or environment configuration can prevent PlatformIO from installing or starting correctly on Windows. PlatformIO documents [conflicts between multiple Python interpreters and multiple PlatformIO Core installations](https://docs.platformio.org/en/latest/core/installation/troubleshooting.html). Check these if the error output refers to an unexpected Python installation or cannot import Python modules:

1. In VS Code Settings, keep `platformio-ide.useBuiltinPython` and `platformio-ide.useBuiltinPIOCore` enabled for this course. These select PlatformIO's portable Python when available and its built-in Core; see the [official settings](https://docs.platformio.org/en/latest/integration/ide/vscode.html#platformio-ide-usebuiltinpython).
2. Inspect Windows user and system environment variables. Check `PATH` for obsolete or conflicting Python/PlatformIO entries, and check whether `PYTHONHOME` or `PYTHONPATH` redirects Python to another installation's libraries. [Python documents how these variables change library lookup](https://docs.python.org/3/using/cmdline.html#environment-variables).
3. Correct entries identified as causing the conflict, then close all VS Code windows and reopen VS Code so it inherits the updated environment.

A global Python installation can coexist with PlatformIO. PlatformIO's [Python installation guide](https://docs.platformio.org/en/latest/faq/install-python.html) even recommends adding Python to `PATH` for its standalone setup. Diagnose the interpreter or library conflict before removing environment entries. This Windows troubleshooting is separate from the Ubuntu 24.04 Mbed dependency repair below.

### Interrupted or Incomplete Installation (All Operating Systems)

Keep VS Code open and allow PlatformIO's initial installation to finish. If installation is interrupted, reopen VS Code and allow the installer to retry. If it remains broken, reset its installation data using the procedure below. A full reset is a recovery option on Windows, Linux and macOS; it is not automatically required after every interruption.

1. Save your work, close all VS Code windows, and stop any other running PlatformIO installation or task.
2. Locate the actual PlatformIO Core directory. The usual locations are:

   | Operating system | Usual directory             |
   | ---------------- | --------------------------- |
   | Windows          | `%USERPROFILE%\.platformio` |
   | Linux / macOS    | `~/.platformio`             |

   Check the installer/error output for the active location first. A custom `core_dir` or `PLATFORMIO_CORE_DIR` can change it; some Windows installations use `C:\.platformio`. See the [Core directory documentation](https://docs.platformio.org/en/latest/projectconf/sections/platformio/options/directory/core_dir.html) and [Windows location note](https://docs.platformio.org/en/latest/faq/troubleshooting.html#unicodedecodeerror-non-ascii-characters-found-in-build-environment).
3. For a failure confined to PlatformIO's Python environment, rename the `penv` subfolder to an unused backup name such as `penv.backup`, then reopen VS Code and let PlatformIO recreate it. If installation still fails, capture the error and check its cause, including network access and the Windows environment checks above.
4. For a complete reset, close VS Code again and remove the active Core directory from its original location. Prefer renaming `.platformio` to an unused backup name such as `.platformio.backup`; alternatively, back up any custom content and delete `.platformio`. PlatformIO documents [removing the Core directory as a complete uninstall of its isolated installation](https://docs.platformio.org/en/latest/core/installation/uninstall.html).
5. Reopen VS Code with PlatformIO IDE enabled and let installation complete. Build the project again to download its platform, framework and toolchain as needed.

Resetting the whole Core directory affects the shared PlatformIO installation used by all projects, including installed packages, global libraries and local framework patches. Keep the project source folder: the Core directory `.platformio` is separate from the project's generated `.pio` folder. On Ubuntu 24.04, the Mbed compatibility repair may need to be applied again after the framework is downloaded.

### Project or Build Errors

- Confirm that VS Code has opened the repository folder containing `platformio.ini` and that PlatformIO IDE has finished initialising.
- Check the first error in the terminal. Download errors require checking internet/proxy access; compilation errors require checking the source and configuration.
- For a stale build after changing configuration, use **PlatformIO: Clean** and rebuild.
- On Ubuntu 24.04, for Python errors mentioning missing `distutils` or `imp` modules, use the targeted compatibility instructions below. Keep the full error output if the repair does not address the failure.

### Upload Drive or Serial Port Not Found

- Check the USB cable, ST-LINK connector and operating system's device detection.
- For this project's `mbed` upload protocol, make the board's removable drive available and writable. If automatic drive detection fails, set `upload_port` in `platformio.ini` to that drive's root path or mount point, as described in the [PlatformIO upload-port documentation](https://docs.platformio.org/en/latest/projectconf/sections/env/options/upload/upload_port.html).
- If ST-LINK is detected but no removable drive appears, check whether its mass-storage interface is enabled. ST documents this optional interface in section 7.4.3 of the [Nucleo-64 user manual](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf). The configured `mbed` upload protocol requires that interface.
- On Windows, check the [ST-LINK USB driver](https://www.st.com/en/development-tools/stsw-link009.html) if ST-LINK interfaces are missing.
- On Linux, check the [USB/serial permissions](https://docs.platformio.org/en/latest/core/installation/udev-rules.html). Serial-port permissions and access to the mounted upload drive are separate checks.
- If Monitor reports that a port is busy, close applications holding that port. Check the selected port and baud rate if text is missing or unreadable.

### Mbed Python Compatibility (Linux Only)

**For Linux users only.** This issue was encountered on Ubuntu 24.04. Apply this workaround only if you encounter the corresponding Python dependency errors. Windows users should follow the normal Build/Upload instructions without this repair.

On Ubuntu 24.04, Mbed's bundled Python dependencies can fail with Python 3.12. [Python 3.12 removed `imp` and `distutils`](https://docs.python.org/3/whatsnew/3.12.html#removed); setuptools supplies the latter for this build setup. The following Linux/Bash procedure checks the installed Mbed dependency, installs setuptools into PlatformIO's Python environment and replaces its obsolete `reload` import if needed.

On Ubuntu 24.04, use it for the corresponding Python dependency errors after PlatformIO has installed `framework-mbed`. It assumes the default `~/.platformio` location. If your installation uses a different location, first identify the active PlatformIO Python environment and framework package and adapt both paths.

```bash
(
set -e

~/.platformio/penv/bin/python - <<'PYTHON'
from pathlib import Path
import ast
import shutil
import subprocess
import sys

core = Path.home() / ".platformio"
if Path(sys.prefix).resolve() != (core / "penv").resolve():
    raise SystemExit("ERROR: Run this with PlatformIO's Python environment.")

path = core / (
    "packages/framework-mbed/"
    "platformio/package_deps/py3/past/builtins/misc.py"
)
if not path.is_file():
    raise SystemExit(f"ERROR: Mbed dependency file not found: {path}")

text = path.read_bytes().decode("utf-8")
imports = [
    node for node in ast.walk(ast.parse(text))
    if isinstance(node, ast.ImportFrom)
    and node.module in ("imp", "importlib")
    and any(alias.name == "reload" for alias in node.names)
]
if len(imports) != 1:
    raise SystemExit("ERROR: Expected exactly one reload import; no changes made.")

node = imports[0]
lines = text.splitlines(keepends=True)
expected = f"from {node.module} import reload"
line = lines[node.lineno - 1]
if (node.end_lineno != node.lineno
        or line.strip().split("#", 1)[0].strip() != expected):
    raise SystemExit("ERROR: Unexpected import format; no changes made.")

# Validate the file before installing a dependency or changing the package.
subprocess.run([sys.executable, "-m", "pip", "install", "setuptools"], check=True)

if node.module == "imp":
    lines[node.lineno - 1] = line.replace(
        "from imp import reload", "from importlib import reload", 1
    )
    patched = "".join(lines)
    ast.parse(patched)
    backup = path.with_suffix(".py.bak")
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_bytes(patched.encode("utf-8"))
    print("Import patched; backup is misc.py.bak (existing backup preserved).")
else:
    print("Import already patched.")
PYTHON

echo "PlatformIO compatibility setup completed."
)
```

This is a reviewed revision of the originally recorded repair: it checks Python import statements rather than matching comments, validates the target before installing setuptools, preserves file line endings and keeps an existing `misc.py.bak`. The setuptools requirement is unpinned; reinstalling or updating the framework package may overwrite the patch. After the command succeeds, retry **PlatformIO: Build** to verify the complete build.

This command uses Bash syntax and the default Linux installation paths. Use this targeted repair for the Ubuntu 24.04 dependency errors described above. The [installation reset](#interrupted-or-incomplete-installation-all-operating-systems) above addresses persistent installation failures.

## References

- [PlatformIO IDE actions and project tasks](https://docs.platformio.org/en/latest/integration/ide/vscode.html).
- [PlatformIO Nucleo-F446RE board documentation](https://docs.platformio.org/en/latest/boards/ststm32/nucleo_f446re.html).
