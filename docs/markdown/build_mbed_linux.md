# Build Mbed OS Projects on Ubuntu or Windows Subsystem for Linux with VS Code

Tested on Ubuntu 20.04 and Ubuntu 24.04. Last tested on 30.12.2024.

## Set up Conda environment for the Build Tools

### Install Miniforge (Conda)

Just answer all prompts with `yes` and/or `Enter`.

```bash
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

### Update Conda

```bash
conda update --all
```

### Disable Conda's auto-activation (optional)

```bash
conda config --set auto_activate_base false
```

## Install Conda environment for Mbed

To replicate the Conda environment, use the file `docs/conda_envs/mbed-env-linux.yml` in this repository.

You can create the environment with the following command

```bash
conda env create -f mbed-env-linux.yml
```

## GCC ARM Embedded Toolchain

### Download the Latest Version of the Toolchain

```bash
ARM_TOOLCHAIN_VERSION=$(curl -s https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads | grep -Po '<h4>Version \K.+(?=</h4>)')

curl -Lo gcc-arm-none-eabi.tar.xz "https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN_VERSION}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN_VERSION}-x86_64-arm-none-eabi.tar.xz"
```

### Get the Toolchain from the ZHAW Server

Under the path

```bash
smb://public.zhaw.ch/staff/pmic/out/Mbed/Linux/gcc-arm-none-eabi
```

you can find the GCC ARM Embedded Toolchain: `gcc-arm-none-eabi.tar.xz`


### Install the GCC ARM Embedded Toolchain

For the following command it is assumed that you have the file `gcc-arm-none-eabi.tar.xz` in the current working directory.

```bash
sudo mkdir /opt/gcc-arm-none-eabi
sudo tar xf gcc-arm-none-eabi.tar.xz --strip-components=1 -C /opt/gcc-arm-none-eabi
echo 'export PATH=$PATH:/opt/gcc-arm-none-eabi/bin' | sudo tee -a /etc/profile.d/gcc-arm-none-eabi.sh
```

## Install Node.js, npm and Mbed VS Code Generator

```bash
sudo apt install -y nodejs
sudo apt install -y npm
sudo npm install -y mbed-vscode-generator -g
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
mbed config GCC_ARM_PATH "/opt/gcc-arm-none-eabi/bin"
mbed-vscode-generator -m NUCLEO_F446RE
```

Replace the `gcc-x64` with `gcc-arm` in the `.vscode/c_cpp_properties.json` and `--profile=debug` with `--profile=develop` in the `.vscode/tasks.json` files

```bash
sed -i 's/"gcc-x64"/"gcc-arm"/g' .vscode/c_cpp_properties.json
sed -i 's/--profile=debug/--profile=develop/g' .vscode/tasks.json
```

## Install the Mbed OS

If you only use VSCode, you can install Mbed OS using git with the following command

```bash
git clone --branch mbed-os-6.17.0 --depth 1 https://github.com/ARMmbed/mbed-os.git
```

## Build the project

Press Ctrl+Shift+B and select `Build Mbed OS application`.

The binary file will be in the `BUILD/NUCLEO_F446RE/GCC_ARM-DEVELOP/PES_Board.bin` directory.

## Flash the binary file to the board

You can flash the binary file to the board using the USB cable. Just drag and drop the binary file to the board when connected.

## Serial Monitor in VS Code

You can use the extension

- `Serial Monitor` by Microsoft

to monitor the serial output of the board in VS Code directly.

## Building in WSL (Windows Subsystem for Linux)

You need to install the extension

- `WSL` by Microsoft

activate it and open the folder in WSL.

<!-- ## Using a Symbolic Link to `mbed-os` to save Space

To save disk space, it is recommended to host only one physical copy of the Mbed OS on your computer. Instead of duplicating the Mbed OS directory for every project, you can create a symbolic link in each project folder that points to a shared mbed-os directory.

For example, the following command will create a symbolic link in the Mbed Programs/PES_Board directory that points to the shared Mbed Programs/mbed-os directory. This allows multiple Mbed projects to share the same copy of Mbed OS.

Command to Create the Symbolic Link below. You have to substitute the paths with your own paths. Here `mbed-os` is the shared directory and `PES_Board` is the project directory.

```bash
ln -s /home/pichim/Mbed\ Programs/mbed-os /home/pichim/Mbed\ Programs/PES_Board/mbed-os
``` -->
