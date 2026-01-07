#!/bin/bash
# HAPS Glider - SITL Test Script
# Launches ArduPilot SITL for testing the AI agent

set -e

# Configuration
VEHICLE="ArduPlane"
FRAME="plane"
LOCATION="HAPS_TEST"  # Custom location for stratospheric testing
MAP_MODULE="--map"
CONSOLE_MODULE="--console"

# Custom location for high-altitude testing (Sydney area, high altitude start)
# Format: lat,lon,alt,heading
CUSTOM_LOCATION="-33.8688,151.2093,100,0"

echo "==========================================="
echo "    HAPS Glider - SITL Test Environment   "
echo "==========================================="
echo ""

# Check if ArduPilot is installed
if ! command -v sim_vehicle.py &> /dev/null; then
    echo "ERROR: ArduPilot SITL not found!"
    echo ""
    echo "Please install ArduPilot SITL:"
    echo "  git clone https://github.com/ArduPilot/ardupilot.git"
    echo "  cd ardupilot"
    echo "  git submodule update --init --recursive"
    echo "  Tools/environment_install/install-prereqs-ubuntu.sh -y"
    echo "  . ~/.profile"
    echo ""
    exit 1
fi

echo "Starting ArduPilot SITL..."
echo "  Vehicle: $VEHICLE"
echo "  Frame: $FRAME"
echo "  Location: $CUSTOM_LOCATION"
echo ""

# Launch SITL
sim_vehicle.py -v $VEHICLE \
    --frame $FRAME \
    --custom-location=$CUSTOM_LOCATION \
    $MAP_MODULE \
    $CONSOLE_MODULE \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    -w  # Wipe EEPROM for clean start

# Note: After SITL starts, in MAVProxy console run:
# param load ../../config/haps_sitl.param
# to load HAPS-specific parameters
