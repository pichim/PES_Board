# Build Mbed OS Projects on Windows with VS Code

Tested on Windows 11. Last tested on 28.01.2024.

## Set up Conda environment for the Build Tools

### Install Miniforge (Conda)

Download and install the latest Windows installer from https://github.com/conda-forge/miniforge. Under Install -> Windows is a link that downloads the installer.

Just answer all prompts with `yes` and/or `Enter`.

### Update Conda

```bash
conda update --all
```

### Disable Conda's auto-activation (optional)

```bash
conda config --set auto_activate_base false
```

## Install Conda environment for Mbed

To replicate the Conda environment, use the file `docs/conda_envs/mbed-env-windows.yml` in this repository.

You can create the environment with the following command

```bash
conda env create -f mbed-env-linux.yml
```

## GCC ARM Embedded Toolchain

### Download the Latest Version of the Toolchain

Download and install the latest installer from https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads. Make sure you select the version of the `GNU Arm Embedded Toolchain` for your operating system, e.g. `arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi.exe` for Windows.

### Get the Toolchain from the ZHAW Server

Under the path `\\shared.zhaw.ch\public\staff\pmic\out\Mbed\Windows` you can find the GCC ARM Embedded Toolchain installer `arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi.exe`.

### Install the GCC ARM Embedded Toolchain

Simply execute the installer and follow the instructions, use the default settings.

## Install Node.js, Npm and Mbed VSCode Generator

Download the latest installer from https://nodejs.org/en/download or get the file `node-v22.13.1-x64.msi` from the ZHAW server `\\shared.zhaw.ch\public\staff\pmic\out\Mbed\Windows`.

Install Node.js, use the default settings. To verify the installation, open a terminal and type

```bash
node -v
npm -v
```

Within the opened terminal install the Mbed VS Code Generator with the following command

```bash
npm install -g mbed-vscode-generator
```

## Clone the Mbed project

Replace `pichim` with your GitHub username so that you clone your fork of the repository (if you intend to use version control).

```bash
git clone https://github.com/pichim/PES_Board.git
```

## Recommended Extensions in VS Code (might not all be necessary)

- `C/C++ Extension Pack` by Microsoft
- `Python` by Microsoft
- `Pylaunch` by Microsoft
- `Black Formatter` by Microsoft
- `Python Debugger` by Microsoft
- `CMake` by twxs
- `CMake Tools` by Microsoft
- `Makefile Tools` by Microsoft

## Activate the Conda environment in VS Code

You have to open the folder of the Mbed project in VS Code.

Press Ctrl+Shift+P and type `Python: Select Interpreter` and select the `mbed-env` environment.

## Set up the Mbed Tools

Make sure you have a file '.mbed' in the root of your project with the following content

```bash
ROOT=.

```

Use the terminal in VS Code for the following commands.

```bash
mbed toolchain GCC_ARM
mbed config GCC_ARM_PATH "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.3 rel1\bin"
mbed-vscode-generator -m NUCLEO_F446RE
```

The following command only works if you use Windows PowerShell. Replace the `gcc-x64` with `gcc-arm` in the `.vscode/c_cpp_properties.json` and `--profile=debug` with `--profile=develop` in the `.vscode/tasks.json` files

```bash
(Get-Content .vscode/c_cpp_properties.json) -replace '"gcc-x64"', '"gcc-arm"' | Set-Content .vscode/c_cpp_properties.json
(Get-Content .vscode/tasks.json) -replace '--profile=debug', '--profile=develop' | Set-Content .vscode/tasks.json
```

## Install the Mbed OS

If you only use VSCode, you can install Mbed OS using git with the following command

```bash
git clone --branch mbed-os-6.17.0 --depth 1 https://github.com/ARMmbed/mbed-os.git
```

## Build the project

Press Ctrl+Shift+B and select `Build Mbed OS application`.

The binary file will be in the `BUILD\NUCLEO_F446RE\GCC_ARM-DEVELOP\PES_Board.bin` directory.

## Flash the binary file to the board

You can flash the binary file to the board using the USB cable. Just drag and drop the binary file to the board when connected.

## Serial Monitor in VS Code

You can use the extension

- `Serial Monitor` by Microsoft

to monitor the serial output of the board in VS Code directly.

<!-- ## Using a Symbolic Link to `mbed-os` to save Space

To save disk space, it is recommended to host only one physical copy of the Mbed OS on your computer. Instead of duplicating the Mbed OS directory for every project, you can create a symbolic link in each project folder that points to a shared mbed-os directory.

For example, the following command will create a symbolic link in the Mbed Programs/PES_Board directory that points to the shared Mbed Programs/mbed-os directory. This allows multiple Mbed projects to share the same copy of Mbed OS.

To make this work you have to run PowerShell as Administrator in navigate to the root of your Mbed project.

Command to Create the Symbolic Link

```bash
New-Item -ItemType SymbolicLink `
    -Path ".\mbed-os" `
    -Value "..\mbed-os"
``` -->

## Building in WSL (Windows Subsystem for Linux, Recommended)

Tests have shown that building the Mbed project in WSL is a lot faster than building it in Windows directly.

You need to install the extension

- `WSL` by Microsoft

activate it and open the folder in WSL. Follow the instructions in the [build_mbed_linux.md](build_mbed_linux.md) file.
