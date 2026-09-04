#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
#

set -euxo pipefail


OS=$(uname -s | tr '[:upper:]' '[:lower:]')

if [[ ${OS} == "linux" ]]; then
    sudo apt-get update
    sudo apt-get install -y wget gpg lsb-release

    # Add Kitware repository for up-to-date CMake if on Ubuntu
    # This is required for the publish step to work
    # Skip if any kitware source already exists (e.g. cpp-sdk-conan images):
    # a second list with a different Signed-By keyring makes apt error out.
    if lsb_release -is | grep -q "Ubuntu" && ! grep -Rqs "apt.kitware.com" /etc/apt/sources.list.d/; then
        sudo wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
        sudo echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
        sudo apt-get update
    fi

    sudo apt-get install -y \
        python3 \
        python3-venv \
        python3-pip \
        cmake \
        cmake-data \
        autoconf \
        automake \
        build-essential \
        ca-certificates \
        curl \
        doxygen \
        g++ \
        git \
        gdb \
        gnupg \
        less \
        libssl-dev \
        libudev-dev \
        ninja-build \
        pkg-config \
        software-properties-common
elif [[ ${OS} == "darwin" ]]; then
    if ! command -v brew >/dev/null 2>&1; then
        echo "Homebrew not found. Please install it first: https://brew.sh/"
        exit 1
    fi
    # On macOS, these are typically enough for the build
    brew install cmake pkg-config libusb ninja python
fi

# Check python3 availability
if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 not found in PATH. Aborting." >&2
  exit 1
fi

# Check venv module
if ! python3 -m venv --help >/dev/null 2>&1; then
  echo "python3 venv module not available. Try: sudo apt-get install --reinstall python3-venv python3-full python3-pip" >&2
  exit 1
fi


if [ ! -f "./venv/bin/activate" ]; then
  echo 'creating and sourcing virtual env'
  if ! python3 -m venv venv; then
    echo "Failed to create venv. Trying with full path to python3."
    if ! /usr/bin/python3 -m venv venv; then
      echo "Failed to create venv with /usr/bin/python3. Aborting." >&2
      exit 1
    fi
  fi
  source ./venv/bin/activate
else
  echo 'sourcing virtual env'
  source ./venv/bin/activate
fi


# Set up conan
if [ ! -f "./venv/bin/conan" ]; then
  echo 'installing conan'
  . ./venv/bin/activate
  pip install --upgrade pip
  pip install conan
fi

conan profile detect || echo "Conan is already installed"
conan remote add viamconan https://viam.jfrog.io/artifactory/api/conan/viamconan --index 0 || echo "Viam conan remote already exists"
