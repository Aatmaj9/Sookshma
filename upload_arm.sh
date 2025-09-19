#!/usr/bin/env bash
set -euo pipefail

# Folder containing your .ino sketch
SKETCH_DIR="${SKETCH_DIR:-./Ard/sookshma}"

cd "$SKETCH_DIR"
compgen -G "*.ino" >/dev/null || { echo "❌ No .ino in $SKETCH_DIR"; exit 2; }

PORT=""; FQBN=""

# --- helper: strict sysfs fallback (Arduino-only: Arduino Due PIDs only) ---
strict_sysfs_pick() {
  # Returns "PORT FQBN" or nothing
  for p in /dev/ttyACM* /dev/ttyUSB*; do
    [[ -e "$p" ]] || continue
    # Find USB device in sysfs and read VID/PID
    dev="$(readlink -f "/sys/class/tty/$(basename "$p")/device" || true)"
    [[ -n "$dev" ]] || continue
    found=""
    for up in "" "/.." "/../.."; do
      vid_file="$dev$up/idVendor"
      pid_file="$dev$up/idProduct"
      [[ -f "$vid_file" && -f "$pid_file" ]] || continue
      VID="$(tr '[:lower:]' '[:upper:]' < "$vid_file" 2>/dev/null || true)"
      PID="$(tr '[:lower:]' '[:upper:]' < "$pid_file" 2>/dev/null || true)"
      # Arduino VIDs: 2341, 2A03. Map only DUE PIDs here.
      if [[ "$VID" =~ ^(2341|2A03)$ ]]; then
        # Due Programming: 0x003D ; Due Native: 0x003E
        if [[ "$PID" == "003D" ]]; then
          echo "$p per1234:sam:arduino_due_x_dbg"
          return 0
        elif [[ "$PID" == "003E" ]]; then
          echo "$p per1234:sam:arduino_due_x"
          return 0
        fi
      fi
      found=1
    done
    [[ -n "$found" ]] || true
  done
  return 1
}

# Fallback: Arduino-strict via VID/PID (Due only)
read -r PORT FQBN <<<"$(strict_sysfs_pick || true)"

[[ -n "$PORT" && -n "$FQBN" ]] || { echo "❌ No Arduino-recognized port/FQBN found."; exit 3; }


echo "🔌 PORT=$PORT"
echo "🧩 FQBN=$FQBN"
echo "📁 SKETCH=$SKETCH_DIR"

# Ensure the core is installed
CORE="${FQBN%:*}"  # e.g., arduino:sam
arduino-cli core list | grep -q "^$CORE\b" || {
  arduino-cli core update-index
  arduino-cli core install "$CORE"
}

echo "🛠️  Compiling..."
arduino-cli compile --fqbn "$FQBN" .

echo "🚀 Uploading..."
arduino-cli upload -p "$PORT" --fqbn "$FQBN" .
echo "✅ Upload complete."
