# -----------------------------------------------------------------------------
# Unitree Actuator SDK compatible build driver
#
# - Drives the CMake/Ninja build
# - Optionally builds Python bindings (pybind11)
# - Can build Python wheels via cibuildwheel (pyproject.toml)
# -----------------------------------------------------------------------------

BUILD ?= debug
BUILD_DIR := build
CMAKE ?= cmake
NINJA ?= ninja
PYTHON ?= python3

# Set PYTHON_BINDINGS=1 to enable building the local Python module via CMake.
PYTHON_BINDINGS ?= 0

ifdef VERBOSE
  NINJA_FLAGS := -v
endif

ifeq ($(BUILD),release)
  # ok
else ifeq ($(BUILD),debug)
  # ok
else
  $(error BUILD must be either 'debug' or 'release')
endif

ifeq ($(OS),Windows_NT)
  HOST_OS := Windows
else
  HOST_OS := $(shell uname)
endif

ifeq ($(HOST_OS),Linux)
  HOST_CPU := $(shell uname -m)
  OUTPUT_TARGET := $(HOST_CPU)-unknown-linux-gnu
else ifeq ($(HOST_OS),Darwin)
  HOST_CPU := $(shell uname -m)
  OUTPUT_TARGET := $(HOST_CPU)-apple-darwin
else
  HOST_CPU := $(shell uname -m)
  OUTPUT_TARGET := $(HOST_CPU)-unknown-$(HOST_OS)
endif

OUTPUT_DIR := $(BUILD_DIR)/$(OUTPUT_TARGET)/$(BUILD)

CMAKE_FLAGS :=

ifeq ($(PYTHON_BINDINGS),1)
  CMAKE_FLAGS += -DUNITREE_BUILD_PYTHON=ON
endif

.PHONY: all configure python clean wheels wheel-local

all: $(OUTPUT_DIR)/CMakeCache.txt
	$(NINJA) -C $(OUTPUT_DIR) $(NINJA_FLAGS)

python:
	@$(MAKE) all PYTHON_BINDINGS=1

configure: $(OUTPUT_DIR)/CMakeCache.txt

$(OUTPUT_DIR)/CMakeCache.txt:
	@$(CMAKE) -E make_directory $(OUTPUT_DIR)
	$(CMAKE) -B "$(OUTPUT_DIR)" -DCMAKE_BUILD_TYPE=$(BUILD) -G Ninja $(CMAKE_FLAGS)

WHEEL_DIR := $(BUILD_DIR)/wheels

$(WHEEL_DIR): export PATH := $(HOME)/.local/bin:$(PATH)

$(WHEEL_DIR):
	@$(PYTHON) -m pip install --upgrade pip cibuildwheel
	@$(PYTHON) -m cibuildwheel --output-dir $(WHEEL_DIR) .

wheels: $(WHEEL_DIR)

# Local (non-manylinux) wheel build helper.
wheel-local:
	@$(PYTHON) -m pip install --upgrade pip build
	@$(PYTHON) -m build --wheel --outdir $(WHEEL_DIR)

clean:
	@$(CMAKE) -E rm -rf "$(BUILD_DIR)"
