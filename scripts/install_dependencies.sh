#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
THIRD_PARTY_DIR="${PROJECT_ROOT}/third_party"
ARDUPILOT_DIR="${THIRD_PARTY_DIR}/ardupilot"
ARDUPILOT_GAZEBO_DIR="${THIRD_PARTY_DIR}/ardupilot_gazebo"
PROJECT_VENV="${PROJECT_ROOT}/.venv"
ARDUPILOT_VENV="${ARDUPILOT_DIR}/.venv"

log() {
    printf '\n[%s] %s\n' "$(date +'%H:%M:%S')" "$*"
}

require_ubuntu_noble() {
    if [[ ! -r /etc/os-release ]]; then
        echo "Cannot identify the operating system." >&2
        exit 1
    fi

    # shellcheck disable=SC1091
    source /etc/os-release
    if [[ "${ID:-}" != "ubuntu" || "${VERSION_CODENAME:-}" != "noble" ]]; then
        echo "This installer currently supports Ubuntu 24.04 (Noble) only." >&2
        exit 1
    fi
}

install_apt_dependencies() {
    local packages=(
        apt-transport-https
        astyle
        autoconf
        automake
        build-essential
        ccache
        clang-format
        cmake
        cppcheck
        curl
        g++
        gawk
        g++-arm-linux-gnueabihf
        gcovr
        git
        jq
        lcov
        libcsfml-audio2.6
        libcsfml-dev
        libcsfml-graphics2.6
        libcsfml-network2.6
        libcsfml-system2.6
        libcsfml-window2.6
        libcurl4-openssl-dev
        libeigen3-dev
        libfmt-dev
        libfreetype6-dev
        libgrpc++-dev
        libgstreamer1.0-dev
        libgstreamer-plugins-base1.0-dev
        libgtk-3-dev
        libgtest-dev
        libopencv-dev
        libpng16-16
        libprotobuf-dev
        libsfml-audio2.6
        libsfml-dev
        libsfml-graphics2.6
        libsfml-network2.6
        libsfml-system2.6
        libsfml-window2.6
        libspdlog-dev
        libsqlite3-dev
        libssl-dev
        libtool
        libtool-bin
        libwxgtk3.2-dev
        libyaml-cpp-dev
        make
        ninja-build
        pkg-config
        ppp
        protobuf-compiler
        protobuf-compiler-grpc
        python3-dev
        python3-pexpect
        python3-pip
        python3-venv
        python3-wxgtk4.0
        rapidjson-dev
        screen
        unzip
        valgrind
        wget
        xfonts-base
        xterm
        zip
    )

    log "Refreshing Ubuntu package metadata"
    sudo apt-get update

    log "Installing system, C++, Python, MAVProxy GUI, and simulator build dependencies"
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${packages[@]}"
}

configure_gazebo_repository() {
    local keyring="/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg"
    local source_file="/etc/apt/sources.list.d/gazebo-stable.list"
    local source_line

    source_line="deb [arch=$(dpkg --print-architecture) signed-by=${keyring}] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main"

    if [[ ! -f "${keyring}" ]]; then
        log "Adding the OSRF Gazebo signing key"
        curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /tmp/icarus-gazebo.gpg
        sudo install -m 0644 /tmp/icarus-gazebo.gpg "${keyring}"
    fi

    if [[ ! -f "${source_file}" ]] || ! grep -Fqx "${source_line}" "${source_file}"; then
        log "Adding the OSRF Gazebo Harmonic repository"
        printf '%s\n' "${source_line}" | sudo tee "${source_file}" >/dev/null
    fi

    log "Installing Gazebo Harmonic"
    sudo apt-get update
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y gz-harmonic libgz-sim8-dev
}

clone_or_update_repository() {
    local url="$1"
    local destination="$2"

    if [[ -d "${destination}/.git" ]]; then
        log "Updating $(basename "${destination}")"
        git -C "${destination}" pull --ff-only
        git -C "${destination}" submodule update --init --recursive
    else
        log "Cloning $(basename "${destination}")"
        git clone --depth 1 --recurse-submodules --shallow-submodules \
            "${url}" "${destination}"
    fi
}

install_ardupilot_python_environment() {
    log "Creating the isolated ArduPilot Python environment"
    python3 -m venv --system-site-packages "${ARDUPILOT_VENV}"
    "${ARDUPILOT_VENV}/bin/python" -m pip install --upgrade pip packaging setuptools wheel attrdict3
    "${ARDUPILOT_VENV}/bin/python" -m pip install --upgrade \
        'empy==3.3.4' \
        dronecan \
        flake8 \
        geocoder \
        intelhex \
        junitparser \
        lxml \
        matplotlib \
        MAVProxy \
        numpy \
        opencv-python \
        psutil \
        ptyprocess \
        pygame \
        pymavlink \
        pyparsing \
        pyserial \
        pyyaml \
        scipy \
        tabulate \
        wsproto
}

build_ardupilot_sitl() {
    log "Building ArduCopter SITL"
    (
        cd "${ARDUPILOT_DIR}"
        PATH="${ARDUPILOT_DIR}/Tools/autotest:${ARDUPILOT_VENV}/bin:${PATH}" \
            ./waf configure --board sitl
        PATH="${ARDUPILOT_DIR}/Tools/autotest:${ARDUPILOT_VENV}/bin:${PATH}" \
            ./waf copter
    )
}

build_ardupilot_gazebo() {
    log "Building the ArduPilot Gazebo plugin"
    cmake -S "${ARDUPILOT_GAZEBO_DIR}" -B "${ARDUPILOT_GAZEBO_DIR}/build" \
        -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build "${ARDUPILOT_GAZEBO_DIR}/build"
}

install_project_python_environment() {
    log "Creating the Icarus Python environment"
    python3 -m venv "${PROJECT_VENV}"
    "${PROJECT_VENV}/bin/python" -m pip install --upgrade pip setuptools wheel
    "${PROJECT_VENV}/bin/python" -m pip install -r "${PROJECT_ROOT}/requirements.txt"
    "${PROJECT_VENV}/bin/python" -m pip check
}

install_ollama() {
    if command -v ollama >/dev/null 2>&1; then
        log "Ollama is already installed"
    else
        log "Downloading and installing Ollama"
        curl -fsSL https://ollama.com/install.sh -o /tmp/icarus-install-ollama.sh
        sh /tmp/icarus-install-ollama.sh
    fi

    log "Downloading the Qwen3 4B model"
    ollama pull qwen3:4b
}

verify_installation() {
    log "Verifying installed components"
    g++ --version | head -n 1
    cmake --version | head -n 1
    protoc --version
    gz sim --versions
    "${ARDUPILOT_VENV}/bin/python" -c 'import pymavlink; import MAVProxy; print("ArduPilot Python dependencies: OK")'
    "${PROJECT_VENV}/bin/python" -c 'import grpc, torch, transformers, peft, trl; print("DCM Python dependencies: OK")'
    test -x "${ARDUPILOT_DIR}/build/sitl/bin/arducopter"
    ollama list
}

main() {
    require_ubuntu_noble
    log "Requesting sudo authentication"
    sudo -v

    mkdir -p "${THIRD_PARTY_DIR}"
    install_apt_dependencies
    configure_gazebo_repository

    clone_or_update_repository \
        https://github.com/ArduPilot/ardupilot.git \
        "${ARDUPILOT_DIR}"
    install_ardupilot_python_environment
    build_ardupilot_sitl

    clone_or_update_repository \
        https://github.com/ArduPilot/ardupilot_gazebo.git \
        "${ARDUPILOT_GAZEBO_DIR}"
    build_ardupilot_gazebo

    install_project_python_environment
    install_ollama
    verify_installation

    log "Icarus dependency installation completed successfully"
}

main "$@"
