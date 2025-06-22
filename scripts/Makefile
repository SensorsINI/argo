# Makefile for building RTIMULib2 and its Python bindings
# This Makefile assumes a Debian-based system (like Raspberry Pi OS or Armbian)
# and Python 3.

.PHONY: all imu clean install_deps

RTIMULIB2_REPO := https://github.com/orca-io/RTIMULib2.git
RTIMULIB2_DIR := RTIMULib2
BUILD_DIR := $(RTIMULIB2_DIR)/build
PYTHON_BINDINGS_DIR := $(RTIMULIB2_DIR)/Linux/python

all: imu

# The main target: builds and installs the RTIMU Python module
imu: $(PYTHON_BINDINGS_DIR)/RTIMU.so

# Step 1: Clone the RTIMULib2 repository if it doesn't exist
$(RTIMULIB2_DIR):
	@echo "Cloning RTIMULib2 repository..."
	git clone $(RTIMULIB2_REPO) $(RTIMULIB2_DIR)
	@echo "RTIMULib2 cloned."

# Step 2: Create a build directory for CMake
$(BUILD_DIR): $(RTIMULIB2_DIR)
	@echo "Creating build directory for RTIMULib2..."
	mkdir -p $(BUILD_DIR)
	@echo "Build directory created."

# Step 3: Run CMake to configure the build
$(BUILD_DIR)/Makefile: $(BUILD_DIR)
	@echo "Running CMake for RTIMULib2..."
	cd $(BUILD_DIR) && cmake ..
	@echo "CMake finished."

# Step 4: Build the RTIMULib2 C++ library
$(BUILD_DIR)/RTIMULib.a: $(BUILD_DIR)/Makefile
	@echo "Building RTIMULib2 C++ library..."
	cd $(BUILD_DIR) && make
	@echo "RTIMULib2 C++ library built."

# Step 5: Install the C++ library system-wide (often a prerequisite for Python bindings)
install_rtimulib_cpp: $(BUILD_DIR)/RTIMULib.a
	@echo "Installing RTIMULib2 C++ library (requires sudo)..."
	sudo make -C $(BUILD_DIR) install
	@echo "RTIMULib2 C++ library installed."

# Step 6: Build and install the RTIMU Python bindings
$(PYTHON_BINDINGS_DIR)/RTIMU.so: install_rtimulib_cpp
	@echo "Building and installing RTIMULib2 Python bindings (requires sudo)..."
	cd $(PYTHON_BINDINGS_DIR) && sudo python3 setup.py install
	@echo "RTIMULib2 Python bindings installed."

# Clean target: removes the cloned repository and all build artifacts
clean:
	@echo "Cleaning RTIMULib2 build artifacts and cloned repository..."
	rm -rf $(RTIMULIB2_DIR)
	@echo "Clean complete."

# Optional: Target to install necessary build dependencies
install_deps:
	@echo "Installing build dependencies (requires sudo)..."
	sudo apt update
	sudo apt install -y git cmake build-essential python3-dev python3-pip
	@echo "Dependencies installed."