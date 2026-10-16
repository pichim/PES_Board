# Course Setup

Use Visual Studio Code with PlatformIO IDE to build and upload the PES Board project, and Microsoft's Serial Monitor extension to view serial output. PlatformIO builds the existing Mbed OS 6 application for the Nucleo-F446RE. This page covers installation and obtaining the project; the [PlatformIO build guide](build_mbed_platformio.md) covers building, uploading and monitoring it.

## Accounts

- [GitHub](https://github.com/): create an account for your fork and course work. A private email address is recommended.
- [MATLAB](https://matlab.mathworks.com/): use your university account for the university licence.

## Software

Install the following software for your operating system:

- [Git](https://git-scm.com/install/): select the Windows, Linux or macOS instructions. After installation, `git --version` should work in a new terminal.
- [Visual Studio Code](https://code.visualstudio.com/download).
- [PlatformIO IDE](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide): install this extension in VS Code as described below. PlatformIO Core is included with the extension.
- [Serial Monitor by Microsoft](https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor): the recommended extension for viewing the board's serial output.
- [MATLAB](https://matlab.mathworks.com/): required for the course's MATLAB exercises.

On Windows, install the [ST-LINK USB driver (STSW-LINK009)](https://www.st.com/en/development-tools/stsw-link009.html) if the board's ST-LINK interfaces are not recognised. This project uploads through the board's USB drive; see [Upload](build_mbed_platformio.md#upload).

On Linux, follow the [PlatformIO USB/serial permission instructions](https://docs.platformio.org/en/latest/core/installation/udev-rules.html). Install and reload the udev rules, then reconnect the board. If serial access still requires group membership, use the group that owns your serial device and log out and back in after changing membership. On Debian/Ubuntu, also install `python3-venv` if it is missing, as described in the [PlatformIO installation instructions](https://docs.platformio.org/en/latest/integration/ide/vscode.html#installation).

### Install the VS Code Extensions

1. Open VS Code and select **Extensions** in the Activity Bar.
2. Search for **PlatformIO IDE** by PlatformIO (`platformio.platformio-ide`).
3. Select **Install** and keep VS Code open while the extension completes its initial setup. Follow any request to reload VS Code. If setup is interrupted or remains broken, follow [installation recovery](build_mbed_platformio.md#interrupted-or-incomplete-installation-all-operating-systems).
4. In Extensions, search for **Serial Monitor** by **Microsoft** (`ms-vscode.vscode-serial-monitor`) and select **Install**.

On Windows, conflicting Python installations or environment settings can interfere with PlatformIO. If setup fails with Python errors, follow the [Windows environment checks](build_mbed_platformio.md#windows-python-and-environment-conflicts).

Use Microsoft's Serial Monitor extension for the course; follow the [serial-monitor instructions](build_mbed_platformio.md#serial-monitor). The project's [extension recommendations](../../.vscode/extensions.json) recommend PlatformIO IDE and Serial Monitor; the Microsoft C/C++ Extension Pack is not required. The pack is distinct from the individual Microsoft C/C++ extension used for C++ language support.

## GitHub

### Fork the Repository

1. Open the [PES Board repository](https://github.com/pichim/PES_Board) on GitHub and sign in.
2. Select **Fork**, choose your account, and select **Create fork**.

<p align="center">
    <img src="../images/fork.png" alt="Example of the Fork button on a GitHub repository" width="600"/><br/>
    <i>Example of the Fork button; the image shows a different repository.</i>
</p>

Your fork is your own copy on GitHub. Clone it to obtain a local working copy on your computer.

### Clone Your Fork

1. On your fork's GitHub page, select **Code**, then **HTTPS**, and copy the clone URL.
2. In a terminal, change to the directory where you want to keep the project.
3. Run the following command, replacing the example URL with your fork's clone URL:

   ```bash
   git clone https://github.com/YOUR_NICK/NAME_OF_FORKED_REPOSITORY.git
   ```

   The uppercase names are placeholders, not a link to an existing repository.

4. Note the location of the newly created repository folder. This is the project folder you will open in VS Code.

## PlatformIO

Continue with [Open the Project](build_mbed_platformio.md#open-the-project), then build the existing `src/main.cpp` using **PlatformIO: Build**. The repository already contains the board and framework configuration.

Mbed OS 6 supplies the embedded C++ APIs used throughout the workshops. Keep both `platformio.ini` and `mbed_app.json`; their roles are explained in [Project Configuration](build_mbed_platformio.md#project-configuration).
