# Icarus Development Dependencies

This file records the host software required to develop and test the Icarus
drone autonomy stack on Ubuntu 24.04.

## Verified Host

- Ubuntu 24.04 LTS (x86_64)
- AMD Ryzen 7 7435HS
- 16 GiB system RAM
- NVIDIA GeForce RTX 4050 Laptop GPU with 6 GiB VRAM
- NVIDIA driver installed and working
- At least 30 GiB of free disk space recommended

## System Build Tools

- `build-essential`
- `cmake`
- `ninja-build`
- `ccache`
- `autoconf`
- `automake`
- `libtool` and `libtool-bin`
- `clang-format`
- `cppcheck`
- `gdb`
- `valgrind`
- `lcov` and `gcovr`
- `git`
- `curl`, `wget`, `jq`, `unzip`, and `zip`
- `screen`, `xterm`, and `astyle`

## Python Environment

- Python 3.12
- `python3-pip`
- `python3-venv`
- `python3-dev`
- A project virtual environment at `.venv`
- An isolated ArduPilot virtual environment at
  `third_party/ardupilot/.venv`

Python DCM, testing, evaluation, and fine-tuning dependencies are declared in
[`requirements.txt`](../../requirements.txt).

## C++ Autonomy Core

- Protocol Buffers compiler and C++ development libraries
- gRPC C++ compiler plugin and development libraries
- Eigen
- OpenSSL
- libcurl
- SQLite
- yaml-cpp
- fmt
- spdlog
- GoogleTest

Ubuntu packages:

```text
protobuf-compiler
protobuf-compiler-grpc
libprotobuf-dev
libgrpc++-dev
libeigen3-dev
libssl-dev
libcurl4-openssl-dev
libsqlite3-dev
libyaml-cpp-dev
libfmt-dev
libspdlog-dev
libgtest-dev
```

## Flight Simulation

### ArduPilot SITL

- ArduPilot source cloned recursively into `third_party/ardupilot`
- ArduCopter SITL build
- MAVProxy
- pymavlink
- DroneCAN and supporting ArduPilot Python packages

The STM32/Pixhawk cross-compiler is intentionally excluded until physical
Pixhawk firmware work begins.

### Gazebo

- OSRF Gazebo package repository
- Gazebo Harmonic (`gz-harmonic`)
- `libgz-sim8-dev`
- Official ArduPilot Gazebo plugin at `third_party/ardupilot_gazebo`
- OpenCV and GStreamer development/runtime dependencies

ROS 2 is intentionally excluded. The initial ArduPilot Gazebo integration does
not require it.

## Local LLM Inference

- Ollama
- Quantized `qwen3:4b` model

The RTX 4050 has 6 GiB VRAM. The 4B quantized model is the initial inference
target. Qwen 30B is not suitable for this laptop.

## Model Development and Fine-Tuning

Installed in the project `.venv`:

- PyTorch
- Transformers
- Datasets
- Accelerate
- PEFT
- TRL
- bitsandbytes
- TensorBoard
- JupyterLab
- pandas and PyArrow

The laptop is appropriate for dataset creation, evaluation, quantized
inference, and small experiments. Substantial 4B QLoRA work may require a GPU
with more VRAM.

## Deliberately Deferred

- ROS 2
- Docker
- Standalone CUDA toolkit
- TensorRT / TensorRT-LLM
- Mission Planner
- Qwen 30B
- STM32/Pixhawk cross-compilation toolchain

The current NVIDIA driver is sufficient for the PyTorch wheels used by the
Python environment. Jetson-specific CUDA and TensorRT components will be
installed on the Jetson Thor through its matching JetPack release.

## Installation

Run the complete installer from the project root:

```bash
./scripts/install_dependencies.sh
```

The script is designed to be safely rerun. It does not remove ModemManager or
BRLTTY, and it does not make the ArduPilot virtual environment the default for
every shell.
