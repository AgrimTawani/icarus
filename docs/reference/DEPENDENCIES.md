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

Direct Python packages are version-pinned and split by purpose under
`requirements/`: `core.txt`, `dev.txt`, `ml.txt` and `ardupilot.txt`.
`requirements.txt` remains a compatibility aggregate.

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

## Optional Local LLM Inference

- A model runtime such as Ollama (not installed by the Phase 7 bootstrap)
- A configured quantized model selected during the later DCM phase

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
- Standalone CUDA toolkit
- TensorRT / TensorRT-LLM
- Mission Planner
- Qwen 30B
- STM32/Pixhawk cross-compilation toolchain

The current NVIDIA driver is sufficient for the PyTorch wheels used by the
Python environment. Jetson-specific CUDA and TensorRT components will be
installed on the Jetson Thor through its matching JetPack release.

## Installation

Select the smallest environment needed:

```bash
./scripts/bootstrap --profile dev
./scripts/bootstrap --profile simulation
./scripts/bootstrap --profile ml
./scripts/bootstrap --profile all
```

The script is designed to be safely rerun. It does not remove ModemManager or
BRLTTY, download a model, or make the ArduPilot virtual environment the default
for every shell. `./scripts/install_dependencies.sh` remains as an alias for the
`all` profile.
