#!/bin/bash

# Exit immediately on error
set -e

# Path to the sketch
SKETCH_PATH="./Ard/sookshma"

# Fully Qualified Board Name
FQBN="arduino:sam:arduino_due_x_dbg"

# Compile the sketch
arduino-cli compile --fqbn $FQBN "$SKETCH_PATH"

# If successful, print message
echo -e "\n✅ Compilation successful for sketch: $SKETCH_PATH"
