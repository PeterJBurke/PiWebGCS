#!/bin/bash

# ==============================================================================
# Combined Installer: Drone Control System + Mavlink Router v5.13
# Target Platform: Raspberry Pi OS
# Includes: Drone position map update fix, Mission/Fence Download & Plotting,
#           Python App Service, Router Path Correction, Installation Check,
#           Auto Serial Port Config, Single-Job Build, Safe Backup Filenames,
#           Correct Heredoc Quoting, Redefined Helper Functions in Subshell
# ==============================================================================
#
# Changes v5.13:
# - Fixed bug where drone position on map did not update (GLOBAL_POSITION_INT
#   handler missing state change flag).
#
# Changes v5.12:
# - Added Mission/Fence download functionality (request list, items, handle ACKs).
# - Added Gevent timeout for mission download.
# - Added backend state management for mission downloads.
# - Modified app.py to parse MISSION_COUNT, MISSION_ITEM_INT, MISSION_ACK.
# - Modified app.py to emit 'mission_update' and 'geofence_update' SocketIO events.
# - Added 'request_mission_download' SocketIO handler in app.py.
# - Added "Load Mission/Fence" and "Fit Mission" buttons to index.html.
# - Added Leaflet Layer Groups and drawing logic for waypoints/polylines and
#   fence polygons in index.html JavaScript.
# - Added styles for waypoint markers in style.css.
# - Updated service description and script versioning.
#
# Changes v5.11:
# - Modified app.py to specifically emit 'HEARTBEAT' messages via the
#   'mavlink_message' socket event to trigger the UI pulse animation.
#
# Usage:
#   1. Save as e.g., install_drone_system_v5.13.sh
#   2. chmod +x install_drone_system_v5.13.sh
#   3. ./install_drone_system_v5.13.sh (uses sudo internally)
#   4. **A REBOOT IS MANDATORY AFTER THE SCRIPT COMPLETES.**
#
# ==============================================================================

set -e # Exit immediately if a command exits with a non-zero status.

# --- Configuration ---
SCRIPT_VERSION="5.13" # <--- New version number
APP_DIR_BASE="drone_control_pi" # Base directory name
SERIAL_PORT="/dev/serial0"
FC_BAUD_RATE="57600" # IMPORTANT: Match your Flight Controller's setting!
MAVLINK_ROUTER_UDP_PORT="14550"
PYTHON_VERSION="python3"
PIP_VERSION="pip3"
CMDLINE_TXT="/boot/cmdline.txt"
CONFIG_TXT="/boot/config.txt"
# Detect correct boot path
if [ -d "/boot/firmware" ] && [ -f "/boot/firmware/cmdline.txt" ] && [ -f "/boot/firmware/config.txt" ]; then
    CMDLINE_TXT="/boot/firmware/cmdline.txt"
    CONFIG_TXT="/boot/firmware/config.txt"
    echo "[INFO] Using boot file location: /boot/firmware/"
else
    echo "[INFO] Using boot file location: /boot/"
fi

# --- Helper Functions ---
print_info() { echo "[INFO] $1"; }
print_warning() { echo "[WARN] $1"; }
print_error() { echo "[ERROR] $1" >&2; }

# --- Check for sudo ---
if ! command -v sudo &> /dev/null; then print_error "sudo command not found."; exit 1; fi
if ! sudo -v; then print_error "Could not acquire sudo privileges."; exit 1; fi

# --- Get User/Group Info ---
if [ -n "$SUDO_USER" ]; then
    INSTALL_USER=$SUDO_USER
    INSTALL_USER_HOME=$(getent passwd $SUDO_USER | cut -d: -f6)
else
    INSTALL_USER=$(whoami)
    INSTALL_USER_HOME=$HOME
fi
INSTALL_GROUP=$(id -gn ${INSTALL_USER})
APP_DIR="${INSTALL_USER_HOME}/${APP_DIR_BASE}"
VENV_PATH="${APP_DIR}/venv"
print_info "Installation Target User: ${INSTALL_USER} (Group: ${INSTALL_GROUP})"
print_info "Application Directory: ${APP_DIR}"
if [ ! -d "${INSTALL_USER_HOME}" ]; then
    print_error "Target user home directory '${INSTALL_USER_HOME}' not found. Exiting."
    exit 1
fi

# --- 1. Install System Dependencies ---
print_info "Updating package lists..."
sudo apt-get update
print_info "Installing essential packages..."
sudo apt-get install -y git python3 python3-pip python3-venv build-essential meson ninja-build pkg-config curl

# --- 2. Prepare Mavlink-Router Source & Check Installation ---
print_info "Cloning/updating mavlink-router repository..."
BUILD_HOME=$HOME # Build in the primary user's home, even if script run via sudo from somewhere else
MAVLINK_ROUTER_SRC_DIR="${BUILD_HOME}/mavlink-router"
# Need to run git commands potentially as the non-root user if BUILD_HOME is their home
if [ "$EUID" -eq 0 ] && [ -n "$SUDO_USER" ] && [ "$INSTALL_USER_HOME" == "$BUILD_HOME" ]; then
    # If running as root via sudo, and building in the sudo user's home, run git as that user
    print_info "Running git operations as user '$INSTALL_USER' in '$BUILD_HOME'..."
    sudo -u $INSTALL_USER git config --global --add safe.directory "${MAVLINK_ROUTER_SRC_DIR}" || print_warning "Failed to set git safe directory."
    if [ -d "${MAVLINK_ROUTER_SRC_DIR}" ]; then
        print_info "mavlink-router directory exists. Attempting pull..."
        sudo -u $INSTALL_USER git -C "${MAVLINK_ROUTER_SRC_DIR}" pull || print_warning "Git pull failed, proceeding with existing code."
    else
        sudo -u $INSTALL_USER git clone https://github.com/intel/mavlink-router.git --recurse-submodules "${MAVLINK_ROUTER_SRC_DIR}"
    fi
    cd "${MAVLINK_ROUTER_SRC_DIR}" # Change directory for subsequent build steps (run as root)
else
    # Run git as the current user (either root or non-root running script directly)
    git config --global --add safe.directory "${MAVLINK_ROUTER_SRC_DIR}" || print_warning "Failed to set git safe directory."
    cd "${BUILD_HOME}"
    if [ -d "${MAVLINK_ROUTER_SRC_DIR}" ]; then
        print_info "mavlink-router directory exists. Attempting pull..."
        cd "${MAVLINK_ROUTER_SRC_DIR}"
        git pull || print_warning "Git pull failed, proceeding with existing code."
    else
        git clone https://github.com/intel/mavlink-router.git --recurse-submodules "${MAVLINK_ROUTER_SRC_DIR}"
        cd "${MAVLINK_ROUTER_SRC_DIR}"
    fi
fi


if ! command -v mavlink-routerd &> /dev/null; then
    print_info "'mavlink-routerd' command not found. Proceeding with build and installation..."
    cd "${MAVLINK_ROUTER_SRC_DIR}" # Ensure we are in the correct directory
    print_info "Building mavlink-router (using single job: -j 1)... This may take a while."
    if [ -d "build" ]; then print_info "Cleaning previous build..."; rm -rf build; fi
    # Ensure meson runs with appropriate permissions if needed, usually okay as root
    meson build --buildtype=release
    ninja -j 1 -C build
    print_info "Installing mavlink-router system-wide (using single job: -j 1)..."
    sudo ninja -j 1 -C build install
    sudo ldconfig
    print_info "mavlink-router build and installation complete."
else
    print_info "'mavlink-routerd' command found. Skipping build and installation."
    INSTALLED_VERSION=$(mavlink-routerd --version 2>/dev/null || echo "unknown")
    print_info "Detected installed mavlink-router version: ${INSTALLED_VERSION}"
    cd "${MAVLINK_ROUTER_SRC_DIR}" # Still change directory for consistency
fi

# --- 3. Configure Mavlink-Router Service ---
print_info "Configuring mavlink-router service files..."
sudo mkdir -p /etc/mavlink-router/
# Use bash -c to ensure redirection works correctly with sudo
sudo bash -c "cat > /etc/mavlink-router/main.conf" << EOF
# Mavlink-Router Configuration v${SCRIPT_VERSION}
[General]
MavlinkSysid = 254 # GCS System ID
UdpEndpoints = gcs_app
UartEndpoints = fc_serial

[UartEndpoint fc_serial]
Device = ${SERIAL_PORT}
Baud = ${FC_BAUD_RATE}
FlowControl = false
# Try adding RTS/CTS flow control if needed and supported by hardware
# RtsCts = true

[UdpEndpoint gcs_app]
Mode = Normal
Address = 127.0.0.1
Port = ${MAVLINK_ROUTER_UDP_PORT}
EOF

# Create systemd service file for mavlink-router
# Use single quotes for EOF to prevent variable expansion inside the service file itself
sudo bash -c "cat > /etc/systemd/system/mavlink-router.service" << 'EOF'
[Unit]
Description=MAVLink Router Service
# Wait for the serial device to be ready
After=network.target systemd-udev-settle.service
Requires=systemd-udev-settle.service

[Service]
WorkingDirectory=/root
# Run mavlink-routerd executable (should be in PATH after install)
ExecStart=/usr/bin/mavlink-routerd -c /etc/mavlink-router/main.conf
# Restart the service if it fails
Restart=always
RestartSec=5
# Log output to systemd journal
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

print_info "Reloading systemd and enabling/restarting mavlink-router service..."
sudo systemctl daemon-reload
sudo systemctl enable mavlink-router.service
# Restart it now to ensure config is loaded, but it might fail until reboot if serial isn't ready
sudo systemctl restart mavlink-router.service
sleep 1 # Give service a moment
# Show status, but don't exit script if it failed (needs reboot)
sudo systemctl status mavlink-router.service --no-pager || print_warning "mavlink-router service status check (may show errors until reboot)."


# --- 4. Automatic Serial Port Configuration ---
print_info "Attempting to automatically configure serial port settings..."
SAFE_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
CONFIG_BACKUP_NEEDED=false # Flag to create backup only once per file

# Check cmdline.txt for console=serial0 or console=ttyS0
print_info "Checking ${CMDLINE_TXT} for serial console..."
if sudo grep -q "console=\(serial0\|ttyS0\)," ${CMDLINE_TXT}; then
    print_info "Serial console found. Backing up and removing..."
    # Create backup with timestamp
    sudo cp ${CMDLINE_TXT} ${CMDLINE_TXT}.bak.${SAFE_TIMESTAMP}
    # Remove the console parameter and the following comma/space
    sudo sed -i.bak 's/console=\(serial0\|ttyS0\),[^ ]* *//g' ${CMDLINE_TXT}
    print_info "Removed serial console parameters from ${CMDLINE_TXT}."
else
    print_info "Serial console not found in ${CMDLINE_TXT}."
fi

# Check config.txt for UART settings
print_info "Checking ${CONFIG_TXT} for UART configuration..."
UART_ENABLED=false
# Check if 'enable_uart=1' exists and is uncommented
if sudo grep -qE "^enable_uart=1" ${CONFIG_TXT}; then
    print_info "UART already enabled in ${CONFIG_TXT}."
    UART_ENABLED=true
# Check if 'enable_uart=1' exists but is commented out
elif sudo grep -qE "^#enable_uart=1" ${CONFIG_TXT}; then
    print_warning "UART setting commented out in ${CONFIG_TXT}. Uncommenting..."
    if [ "$CONFIG_BACKUP_NEEDED" = false ]; then # Backup only if not already done
       sudo cp ${CONFIG_TXT} ${CONFIG_TXT}.bak.${SAFE_TIMESTAMP}
       CONFIG_BACKUP_NEEDED=true
    fi
    sudo sed -i.bak '/^#enable_uart=1/s/^#//' ${CONFIG_TXT}
    UART_ENABLED=true
fi

# If 'enable_uart=1' was not found at all, add it
if [ "$UART_ENABLED" = false ]; then
    print_warning "'enable_uart=1' not found in ${CONFIG_TXT}. Adding..."
    if [ "$CONFIG_BACKUP_NEEDED" = false ]; then # Backup only if not already done
       sudo cp ${CONFIG_TXT} ${CONFIG_TXT}.bak.${SAFE_TIMESTAMP}
       CONFIG_BACKUP_NEEDED=true
    fi
    # Append the setting with a comment
    echo -e "\n# Enable UART for serial communication (added by script v${SCRIPT_VERSION})\nenable_uart=1" | sudo tee -a ${CONFIG_TXT} > /dev/null
fi

# Check if 'dtoverlay=disable-bt' is present (needed for Pi 3/4/ZeroW using /dev/serial0)
if ! sudo grep -qE "^dtoverlay=disable-bt" ${CONFIG_TXT}; then
    print_warning "Adding 'dtoverlay=disable-bt' to ${CONFIG_TXT} to disable Bluetooth conflict..."
    if [ "$CONFIG_BACKUP_NEEDED" = false ]; then # Backup only if not already done
       sudo cp ${CONFIG_TXT} ${CONFIG_TXT}.bak.${SAFE_TIMESTAMP}
       # CONFIG_BACKUP_NEEDED=true # No need to set flag again
    fi
    # Append the setting with a comment
    echo -e "\n# Disable Bluetooth to free up PL011 UART (added by script v${SCRIPT_VERSION})\ndtoverlay=disable-bt" | sudo tee -a ${CONFIG_TXT} > /dev/null
else
    print_info "'dtoverlay=disable-bt' already present in ${CONFIG_TXT}."
fi

print_info "Automatic serial port configuration attempt finished."
print_warning "A REBOOT IS REQUIRED for these serial changes to take effect."


# --- 5. Setup Drone Control Python Application ---
print_info "Setting up Drone Control application directory: ${APP_DIR}"
sudo mkdir -p "${APP_DIR}/templates" "${APP_DIR}/static/css" "${APP_DIR}/static/lib"
# Ensure the target user owns the application directory
sudo chown -R ${INSTALL_USER}:${INSTALL_GROUP} "${APP_DIR}"

# Create virtual environment and install dependencies AS THE TARGET USER
# Use a subshell with sudo -u to run commands as the correct user
sudo -u ${INSTALL_USER} bash << VENV_EOF
# Redefine helper functions for this subshell context
print_info() { echo "[SUB:venv] INFO: \$1"; }
print_warning() { echo "[SUB:venv] WARN: \$1"; }
print_error() { echo "[SUB:venv] ERROR: \$1" >&2; }

print_info "(Running venv creation as user ${INSTALL_USER} in ${APP_DIR})"
cd "${APP_DIR}" || { print_error "Could not cd to ${APP_DIR}"; exit 1; } # Exit subshell on failure

# Check if venv already exists
if [ -d "${VENV_PATH}" ]; then
    print_warning "Virtual environment ${VENV_PATH} already exists. Skipping creation."
else
    print_info "Creating Python virtual environment in ${VENV_PATH}..."
    ${PYTHON_VERSION} -m venv "${VENV_PATH}"
    if [ $? -ne 0 ]; then print_error "Failed to create virtual environment."; exit 1; fi
fi

# Activate venv (not strictly needed, just call pip directly) and install packages
print_info "Installing/updating Python dependencies into ${VENV_PATH}..."
# Ensure pip is up-to-date
"${VENV_PATH}/bin/pip" install --upgrade pip
if [ $? -ne 0 ]; then print_error "Failed to upgrade pip."; exit 1; fi

# Install required packages
"${VENV_PATH}/bin/pip" install Flask pymavlink Flask-SocketIO gevent gevent-websocket
if [ $? -ne 0 ]; then print_error "Failed to install Python packages."; exit 1; fi

print_info "Python dependency installation complete."
VENV_EOF

# Verify venv creation success from the main script
if [ ! -f "${VENV_PATH}/bin/python" ]; then
    print_error "Virtual environment python not found at ${VENV_PATH}/bin/python. Setup failed."
    exit 1
fi

# Download Frontend JS libraries AS THE TARGET USER
print_info "Downloading Frontend JavaScript libraries to ${APP_DIR}/static/lib/ ..."
sudo -u ${INSTALL_USER} curl -sL https://unpkg.com/leaflet@1.9.4/dist/leaflet.css -o "${APP_DIR}/static/lib/leaflet.css"
sudo -u ${INSTALL_USER} curl -sL https://unpkg.com/leaflet@1.9.4/dist/leaflet.js -o "${APP_DIR}/static/lib/leaflet.js"
sudo -u ${INSTALL_USER} curl -sL https://cdn.socket.io/4.7.4/socket.io.min.js -o "${APP_DIR}/static/lib/socket.io.min.js"
print_info "Frontend libraries downloaded."


# --- 6. Create Application Files ---
print_info "Creating ${APP_DIR}/app.py (v${SCRIPT_VERSION})..."
# Needs variable expansion for SCRIPT_VERSION, MAVLINK_CONNECTION_STRING etc -> Use unquoted EOF
# *** THIS IS THE v5.13 VERSION (Includes map update fix) ***
cat > "${APP_DIR}/app.py" << EOF
# Drone Control App v${SCRIPT_VERSION}
# Connects to mavlink-router via UDP
# Added Mission/Fence Download & Plotting
# Fix: GLOBAL_POSITION_INT now sets state_updated flag
from gevent import monkey
monkey.patch_all()
import sys, time, threading, json, gevent, math, traceback, collections
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
from pymavlink import mavutil
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

# --- Configuration ---
SCRIPT_VERSION="${SCRIPT_VERSION}" # Pass script version
MAVLINK_CONNECTION_STRING = 'udp:127.0.0.1:${MAVLINK_ROUTER_UDP_PORT}'
WEB_SERVER_HOST = '0.0.0.0'
WEB_SERVER_PORT = 5000
# --- Constants ---
HEARTBEAT_TIMEOUT = 7
REQUEST_STREAM_RATE_HZ = 4
COMMAND_ACK_TIMEOUT = 5
TELEMETRY_UPDATE_INTERVAL = 0.1 # 10 Hz
MISSION_DOWNLOAD_TIMEOUT = 10 # Seconds to wait for mission items
# ArduPilot Custom Flight Modes & MAV_RESULT mapping
AP_CUSTOM_MODES = { 'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4, 'LOITER': 5, 'RTL': 6, 'LAND': 9, 'POS_HOLD': 16, 'BRAKE': 17, 'THROW': 18, 'AVOID_ADSB': 19, 'GUIDED_NOGPS': 20, 'SMART_RTL': 21, 'FLOWHOLD': 22, 'FOLLOW': 23, 'ZIGZAG': 24, 'SYSTEMID': 25, 'AUTOROTATE': 26, 'AUTO_RTL': 27 }
AP_MODE_NAME_TO_ID = {v: k for k, v in AP_CUSTOM_MODES.items()}
MAV_RESULT_ENUM = mavutil.mavlink.enums['MAV_RESULT']
MAV_RESULT_STR = {v: k for k, v in MAV_RESULT_ENUM.items()}
# MAVLink Command IDs (ensure these are in your mavlink dialect)
try:
    MAV_CMD_NAV_WAYPOINT = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
    MAV_MISSION_TYPE_MISSION = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    MAV_MISSION_TYPE_FENCE = mavutil.mavlink.MAV_MISSION_TYPE_FENCE # Often fence uses mission type
    MAV_MISSION_ACCEPTED = mavutil.mavlink.MAV_MISSION_ACCEPTED
    MAVLINK_MSG_ID_MISSION_REQUEST_LIST = mavutil.mavlink.MAVLINK_MSG_ID_MISSION_REQUEST_LIST
    MAVLINK_MSG_ID_MISSION_COUNT = mavutil.mavlink.MAVLINK_MSG_ID_MISSION_COUNT
    MAVLINK_MSG_ID_MISSION_REQUEST_INT = mavutil.mavlink.MAVLINK_MSG_ID_MISSION_REQUEST_INT
    MAVLINK_MSG_ID_MISSION_ITEM_INT = mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT
    MAVLINK_MSG_ID_MISSION_ACK = mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ACK
except AttributeError as e:
    print(f"ERROR: MAVLink dialect missing required definitions: {e}. Mission features disabled.")
    # Add dummy values to prevent NameErrors later, but functionality will be broken
    MAV_CMD_NAV_WAYPOINT = 16
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5102
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5103
    MAV_MISSION_TYPE_MISSION = 0
    MAV_MISSION_TYPE_FENCE = 1
    MAV_MISSION_ACCEPTED = 0
    MAVLINK_MSG_ID_MISSION_REQUEST_LIST = 43
    MAVLINK_MSG_ID_MISSION_COUNT = 44
    MAVLINK_MSG_ID_MISSION_REQUEST_INT = 51
    MAVLINK_MSG_ID_MISSION_ITEM_INT = 73
    MAVLINK_MSG_ID_MISSION_ACK = 47

FENCE_COMMANDS = {MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION}

# --- Flask & SocketIO Setup ---
app = Flask(__name__)
app.config['SECRET_KEY'] = f'rpi_drone_secret_{SCRIPT_VERSION}!' # Use SCRIPT_VERSION
socketio = SocketIO(app, async_mode='gevent')

# --- Global State ---
mavlink_connection = None; mavlink_thread = None; telemetry_update_thread = None
last_heartbeat_time = 0; drone_state_changed = False; drone_state_lock = threading.Lock()
drone_state = {'connected': False, 'armed': False, 'mode': 'UNKNOWN', 'lat': 0.0, 'lon': 0.0, 'alt_rel': 0.0, 'alt_abs': 0.0, 'heading': 0.0, 'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'airspeed': 0.0, 'groundspeed': 0.0, 'battery_voltage': 0.0, 'battery_remaining': -1, 'battery_current': -1.0, 'gps_fix_type': 0, 'satellites_visible': 0, 'hdop': 99.99, 'system_status': 0, 'pitch': 0.0, 'roll': 0.0, 'home_lat': None, 'home_lon': None, 'ekf_flags': 0, 'ekf_status_report': 'EKF INIT'}
data_streams_requested = False; home_position_requested = False
pending_commands = collections.OrderedDict()

# --- Mission/Fence Download State ---
mission_download_state = {
    "active": False,
    "start_time": 0,
    "expected_count": 0,
    "received_count": 0,
    "waypoints": [],
    "fence_points": [],
    "mission_type": MAV_MISSION_TYPE_MISSION, # Default, might change based on request
    "last_requested_index": -1,
    "timeout_event": None
}
mission_state_lock = threading.Lock()

# --- MAVLink Helper Functions ---
def get_ekf_status_report(flags):
    if not flags: return "EKF N/A"
    if not (flags & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL): return "EKF INIT(Gyro)"
    if not (flags & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION): return "EKF INIT(Att)"
    # Check specific EKF flags if they exist in the dialect
    if hasattr(mavutil.mavlink, 'MAV_SYS_STATUS_EKF_CHECK') and not (flags & mavutil.mavlink.MAV_SYS_STATUS_EKF_CHECK):
         return "EKF Init/Fail" # Generic EKF check failed
    # More detailed checks (optional, based on specific bits in EKF_STATUS_REPORT if available)
    # Example using standard EKF flags (adjust based on actual FC messages)
    # if not (flags & mavutil.mavlink.MAV_EKF_ATTITUDE): return "EKF No Att"
    # if not (flags & mavutil.mavlink.MAV_EKF_VELOCITY_HORIZ): return "EKF No Vel H"
    # if not (flags & mavutil.mavlink.MAV_EKF_VELOCITY_VERT): return "EKF No Vel V"
    # if not (flags & mavutil.mavlink.MAV_EKF_POS_HORIZ_REL): return "EKF No Pos H Rel"
    # if not (flags & mavutil.mavlink.MAV_EKF_POS_HORIZ_ABS): return "EKF No Pos H Abs"
    # if not (flags & mavutil.mavlink.MAV_EKF_POS_VERT_ABS): return "EKF No Pos V Abs"
    # if not (flags & mavutil.mavlink.MAV_EKF_CONST_POS_MODE): # Not in const pos mode
    #     if not (flags & mavutil.mavlink.MAV_EKF_PRED_POS_HORIZ_REL): return "EKF No Pred Pos H Rel"
    #     if not (flags & mavutil.mavlink.MAV_EKF_PRED_POS_HORIZ_ABS): return "EKF No Pred Pos H Abs"
    return "EKF OK"


def reset_mission_download_state(emit_update=False):
     with mission_state_lock:
        if mission_download_state["timeout_event"]:
            mission_download_state["timeout_event"].cancel()
            mission_download_state["timeout_event"] = None
        mission_download_state.update({
            "active": False, "start_time": 0, "expected_count": 0,
            "received_count": 0, "waypoints": [], "fence_points": [],
            "last_requested_index": -1, "mission_type": MAV_MISSION_TYPE_MISSION
        })
        if emit_update:
            # Ensure socketio object is available (might not be during initial setup/error)
            if 'socketio' in globals() and socketio:
                try:
                    socketio.emit('mission_update', [], namespace='/drone')
                    socketio.emit('geofence_update', [], namespace='/drone')
                except Exception as e:
                    print(f"Error emitting mission reset update: {e}")
            else:
                 print("WARN: Cannot emit mission reset update, socketio not ready.")
        print("Mission download state reset.")


def connect_mavlink():
    global mavlink_connection, last_heartbeat_time, data_streams_requested, home_position_requested, pending_commands, drone_state_changed
    reset_mission_download_state() # Reset mission state on reconnect attempt
    if mavlink_connection:
        try:
            print("Closing existing MAVLink connection...")
            mavlink_connection.close()
        except Exception as close_err:
             print(f"Minor error closing existing connection: {close_err}")
        finally:
             mavlink_connection = None
    with drone_state_lock:
        drone_state.update({'connected': False, 'armed': False, 'ekf_status_report': 'INIT'})
        drone_state_changed = True # Ensure UI gets update
    data_streams_requested = False
    home_position_requested = False
    pending_commands.clear()
    try:
        print(f"Connecting via UDP: {MAVLINK_CONNECTION_STRING}...")
        mavlink_connection = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING, source_system=250) # GCS ID 250
        print("Waiting for heartbeat...")
        mavlink_connection.wait_heartbeat(timeout=10) # Wait for the first heartbeat
        if mavlink_connection.target_system == 0:
            raise TimeoutError("No heartbeat received from target system")
        print(f"MAVLink connected! Drone SYSID:{mavlink_connection.target_system} COMPID:{mavlink_connection.target_component}")
        with drone_state_lock:
            drone_state['connected'] = True
            drone_state_changed = True # Ensure UI gets update
        last_heartbeat_time = time.time()
        # Request standard data streams needed for UI
        request_data_streams()
        # Do NOT automatically request mission here - should be user initiated
        return True
    except Exception as e:
        print(f"MAVLink Connect Error: {e}")
        traceback.print_exc()
        with drone_state_lock:
            drone_state.update({'connected': False, 'armed': False, 'ekf_status_report': 'N/A'})
            drone_state_changed = True # Ensure UI gets update
        if mavlink_connection:
            try:
                 mavlink_connection.close()
            except Exception as close_ex:
                 print(f"Error closing connection within exception handler: {close_ex}")
        mavlink_connection = None
        return False

def request_data_streams(req_rate_hz=REQUEST_STREAM_RATE_HZ):
    global data_streams_requested, home_position_requested
    if not mavlink_connection or not drone_state.get("connected", False):
        print("WARN: Cannot request streams, MAVLink not connected.")
        return
    try:
        target_sys = mavlink_connection.target_system
        target_comp = mavlink_connection.target_component
        # Define required message IDs and their desired update rates (Hz)
        msgs = {
            # Core Status & Control
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT: 1,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 1, # Battery, EKF status
            mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_ACK: 5, # Ensure ACKs are received promptly
            mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT: 1, # Flight controller messages
            # PFD & Map Data
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5, # Lat, Lon, Alt (Rel & Abs), HDG
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 2, # Airspeed, Groundspeed (backup), Climb Rate
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 10, # Pitch, Roll for AHRS display
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 1, # GPS Fix type, Satellites, HDOP
            mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION: 0.5, # Request until received
            # Mission/Fence related messages (needed for monitoring/acks, not continuous streaming of items)
            mavutil.mavlink.MAVLINK_MSG_ID_MISSION_COUNT: 1, # Needed to initiate download
            mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT: 0, # DO NOT STREAM Items continuously (set rate to 0 or interval to -1)
            mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ACK: 1, # Need ACKs for mission operations
            # MAVLINK_MSG_ID_MISSION_CURRENT: 1, # Optional: For monitoring current WP in AUTO mode
        }
        print(f"Requesting data streams (Target SYS:{target_sys} COMP:{target_comp})...")
        for msg_id, rate_hz in msgs.items():
            # interval_us: time between messages in microseconds. -1=disable, 0=default rate
            interval_us = int(1e6 / rate_hz) if rate_hz > 0 else -1

            # Special handling: Stop requesting HOME_POSITION once received
            if msg_id == mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION and home_position_requested:
                interval_us = -1 # Disable stream

            # Special handling: Ensure MISSION_ITEM_INT is never streamed continuously
            if msg_id == mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT:
                 interval_us = -1 # Explicitly disable stream rate

            # Send the command if interval is valid or we need to disable home pos req
            if interval_us != -1 or (msg_id == mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION and not home_position_requested):
                 print(f"  -> Requesting MSG ID {msg_id} at interval {interval_us} us ({rate_hz if rate_hz > 0 else 'Disabled'} Hz)")
                 mavlink_connection.mav.command_long_send(
                     target_sys, target_comp,
                     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                     0, # Confirmation
                     msg_id, # param1: Message ID
                     interval_us, # param2: Interval in microseconds
                     0, 0, 0, 0, 0 # param3-7: Reserved
                 )
                 gevent.sleep(0.05) # Short pause between requests
            # else:
            #      print(f"  -> Skipping MSG ID {msg_id} (Rate {rate_hz} Hz results in interval {interval_us} us)")


        data_streams_requested = True
        print("Data stream requests sent.")
    except Exception as e:
        print(f"ERROR Sending stream requests: {e}")
        traceback.print_exc()
        data_streams_requested = False


def check_pending_command_timeouts():
    now = time.time()
    timed_out_cmds = []
    # Use list() to avoid modifying dict during iteration if needed, though we modify after loop
    for cmd_id, data in list(pending_commands.items()):
        if now - data['time'] > COMMAND_ACK_TIMEOUT:
            timed_out_cmds.append(cmd_id)

    for cmd_id in timed_out_cmds:
        if cmd_id in pending_commands: # Check again in case ACK arrived just now
            name = pending_commands[cmd_id]['name']
            print(f"WARN: CMD {name} ({cmd_id}) timed out after {COMMAND_ACK_TIMEOUT}s.")
            try:
                socketio.emit('command_ack_received', {
                    'command_id': cmd_id, 'command_name': name,
                    'result': -1, # Use -1 or a specific timeout code
                    'result_text': 'Timeout'
                }, namespace='/drone')
            except Exception as e:
                 print(f"Error emitting command timeout status: {e}")
            try:
                del pending_commands[cmd_id]
            except KeyError:
                pass # Already deleted

def schedule_mission_timeout():
    with mission_state_lock:
        if mission_download_state["timeout_event"]:
            # Cancel previous timeout if exists
            mission_download_state["timeout_event"].cancel()
        # Schedule the timeout handler to run after MISSION_DOWNLOAD_TIMEOUT seconds
        mission_download_state["timeout_event"] = gevent.spawn_later(MISSION_DOWNLOAD_TIMEOUT, handle_mission_timeout)
        print(f"Mission timeout scheduled for {MISSION_DOWNLOAD_TIMEOUT} seconds.")

def handle_mission_timeout():
    with mission_state_lock:
        if not mission_download_state["active"]:
            print("Mission timeout triggered, but download no longer active.")
            return # Already handled (completed, aborted, or reset)

        # Log the timeout and notify the UI
        last_req = mission_download_state["last_requested_index"]
        expected = mission_download_state["expected_count"]
        received = mission_download_state["received_count"]
        timeout_msg = f"Mission/Fence download timed out after {MISSION_DOWNLOAD_TIMEOUT}s."
        if last_req != -1:
            timeout_msg += f" Waiting for item {last_req} (received {received}/{expected})."
        print(f"ERROR: {timeout_msg}")
        try:
            socketio.emit('status_message', {'text': timeout_msg, 'type': 'error'}, namespace='/drone')
        except Exception as e:
            print(f"Error emitting mission timeout status: {e}")

        # Send a NAK (Not Acknowledged) for the item we were waiting for? Optional.
        # Might help FC clean up its state, but can be complex.
        # if mavlink_connection and last_req != -1:
        #    try:
        #        mavlink_connection.mav.mission_ack_send(
        #            mavlink_connection.target_system, mavlink_connection.target_component,
        #            mavutil.mavlink.MAV_MISSION_ERROR, # Or specific error like MAV_MISSION_OPERATION_TIMEOUT
        #            mission_download_state["mission_type"]
        #        )
        #        print("Sent MISSION_ACK (Error/Timeout) due to timeout.")
        #    except Exception as ack_err:
        #        print(f"Error sending mission NAK on timeout: {ack_err}")

        # Reset the state (clears active flag, lists, etc.) and notify UI to clear display
        reset_mission_download_state(emit_update=True)


def mavlink_receive_loop():
    global last_heartbeat_time, drone_state, mavlink_connection, data_streams_requested, home_position_requested, drone_state_changed
    last_ack_check = time.time()

    while True:
        # --- Connection Check ---
        if not mavlink_connection:
            print("Attempting MAVLink reconnect...")
            if not connect_mavlink():
                # Ensure UI knows mission state is cleared on failed connect
                reset_mission_download_state(emit_update=True)
                gevent.sleep(5) # Wait before retrying connection
                continue
            else:
                 # Connection successful, UI already updated by connect_mavlink
                 pass

        # --- Heartbeat Timeout Check ---
        if drone_state.get('connected') and (time.time() - last_heartbeat_time > HEARTBEAT_TIMEOUT):
            print(f"MAVLink timeout (No heartbeat for >{HEARTBEAT_TIMEOUT}s). Resetting connection.")
            if mavlink_connection:
                try: mavlink_connection.close(); print("Closed timed-out connection.")
                except Exception as e: print(f"Minor error closing on timeout: {e}")
            mavlink_connection = None
            with drone_state_lock:
                drone_state.update({'connected': False, 'armed': False, 'ekf_status_report': 'N/A'})
                drone_state_changed = True
            reset_mission_download_state(emit_update=True) # Clear mission state on disconnect
            data_streams_requested = False # Need to re-request streams on reconnect
            home_position_requested = False
            pending_commands.clear()
            gevent.sleep(1) # Short pause before trying to reconnect
            continue

        # --- Pending Command Timeout Check ---
        if time.time() - last_ack_check > 1.0: # Check timeouts roughly every second
            check_pending_command_timeouts()
            last_ack_check = time.time()

        # --- Receive Message ---
        msg = None
        state_updated = False # Flag for telemetry state changes
        try:
            # Use recv_match with short blocking timeout for responsiveness
            msg = mavlink_connection.recv_match(blocking=True, timeout=0.05)
        except mavutil.mavlink.MAVError as mave:
             print(f"MAVLink Packet Error: {mave}. Ignoring.") # Corrupt packet, try to continue
             gevent.sleep(0.01)
             continue
        except OSError as e:
             # Handle OS errors (e.g., network socket closed unexpectedly)
             print(f"MAVLink Comms OS Error: {e}. Resetting connection.")
             if mavlink_connection:
                 try: mavlink_connection.close(); print("Closed connection after OS error.")
                 except Exception as e_cl: print(f"Minor error closing on OS err: {e_cl}")
             mavlink_connection = None
             with drone_state_lock:
                 drone_state.update({'connected': False, 'armed': False, 'ekf_status_report': 'N/A'})
                 drone_state_changed = True
             reset_mission_download_state(emit_update=True) # Clear mission state
             data_streams_requested = False
             home_position_requested = False
             pending_commands.clear()
             gevent.sleep(5) # Wait before trying to reconnect
             continue
        except Exception as e: # Catch other potential errors during receive
             print(f"Unexpected MAVLink Receive Error: {e}. Resetting connection.")
             traceback.print_exc()
             if mavlink_connection:
                 try: mavlink_connection.close(); print("Closed connection after unexpected error.")
                 except Exception as e_cl: print(f"Minor error closing on unexpected err: {e_cl}")
             mavlink_connection = None
             with drone_state_lock:
                  drone_state.update({'connected': False, 'armed': False, 'ekf_status_report': 'N/A'})
                  drone_state_changed = True
             reset_mission_download_state(emit_update=True) # Clear mission state
             data_streams_requested = False
             home_position_requested = False
             pending_commands.clear()
             gevent.sleep(5) # Wait before trying to reconnect
             continue


        # --- Process Received Message ---
        if msg:
            msg_type = msg.get_type()
            source_sys = msg.get_srcSystem()

            # Only process messages from the connected target system (unless it's a GCS message?)
            # This check prevents processing looped-back messages from our own system (ID 250)
            if mavlink_connection and source_sys != mavlink_connection.target_system:
                # Allow processing ACKs sent *to* our GCS system ID (250)
                is_ack_to_us = msg_type == 'COMMAND_ACK' and getattr(msg, 'target_system', 0) == mavlink_connection.source_system
                # Allow processing mission requests sent *to* our GCS system ID (for upload later)
                is_mission_req_to_us = msg_type == 'MISSION_REQUEST_INT' and getattr(msg, 'target_system', 0) == mavlink_connection.source_system
                # Allow processing mission counts sent *to* our GCS system ID
                is_mission_count_to_us = msg_type == 'MISSION_COUNT' and getattr(msg, 'target_system', 0) == mavlink_connection.source_system
                 # Allow processing mission items sent *to* our GCS system ID
                is_mission_item_to_us = msg_type == 'MISSION_ITEM_INT' and getattr(msg, 'target_system', 0) == mavlink_connection.source_system
                # Allow processing mission ACKs sent *to* our GCS system ID
                is_mission_ack_to_us = msg_type == 'MISSION_ACK' and getattr(msg, 'target_system', 0) == mavlink_connection.source_system


                # Allow HEARTBEAT from any system for diagnostics? Maybe filter later.
                # Let's strictly filter for now, except for messages explicitly directed to the GCS
                if msg_type != 'HEARTBEAT' and not (is_ack_to_us or is_mission_req_to_us or is_mission_count_to_us or is_mission_item_to_us or is_mission_ack_to_us):
                    # print(f"DBG: Ignoring message {msg_type} from system {source_sys} (expected {mavlink_connection.target_system})")
                    continue # Skip processing messages not from the drone unless directed to us


            # Emit ONLY HEARTBEAT from the drone for UI pulse
            if msg_type == 'HEARTBEAT' and mavlink_connection and source_sys == mavlink_connection.target_system:
                try:
                    socketio.emit('mavlink_message', msg.to_dict(), namespace='/drone')
                except Exception as emit_err:
                    print(f"Error emitting heartbeat message: {emit_err}")
            # Do not emit other messages to the dump by default to reduce load

            try:
                # --- Standard Message Parsing ---
                # Use drone_state_lock only when modifying shared drone_state dictionary
                if msg_type == 'HEARTBEAT': # Process heartbeats from target system
                    with drone_state_lock:
                        orig_repr = repr(drone_state) # Check if state actually changes
                        last_heartbeat_time = time.time()
                        was_conn = drone_state["connected"]
                        drone_state['connected'] = True # Mark as connected upon receiving heartbeat
                        drone_state['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        # Determine flight mode (handle custom modes for ArduPilot)
                        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
                            drone_state['mode'] = AP_MODE_NAME_TO_ID.get(msg.custom_mode, f"Custom({msg.custom_mode})")
                        else:
                            # Fallback to standard base_mode string representation if not custom
                            drone_state['mode'] = mavutil.mode_string_v10(msg) # Check if this function exists/works
                        drone_state['system_status'] = msg.system_status # Overall system status state

                        # If we just reconnected or streams weren't requested, request them
                        if (not was_conn or not data_streams_requested):
                            print("Heartbeat received. Requesting data streams...")
                            request_data_streams()
                        if not was_conn:
                            print("Drone Reconnected.")
                            socketio.emit('status_message', {'text': 'Drone Reconnected', 'type': 'info'}, namespace='/drone')
                            # Reset mission state in UI if we just reconnected
                            reset_mission_download_state(emit_update=True)

                        if repr(drone_state) != orig_repr: state_updated = True

                elif msg_type == 'GLOBAL_POSITION_INT':
                    with drone_state_lock:
                         orig_repr = repr(drone_state) # Check if state actually changes
                         drone_state.update({
                            'lat': msg.lat / 1e7 if msg.lat != 0 else 0.0, # Handle 0 case if needed
                            'lon': msg.lon / 1e7 if msg.lon != 0 else 0.0,
                            'alt_rel': msg.relative_alt / 1000.0, # Relative alt in meters
                            'alt_abs': msg.alt / 1000.0, # Absolute alt (MSL) in meters
                            'heading': msg.hdg / 100.0 if msg.hdg != 65535 else drone_state.get('heading', 0.0), # Heading in degrees (0..359.99), 65535=invalid
                            'vx': msg.vx / 100.0, 'vy': msg.vy / 100.0, 'vz': msg.vz / 100.0, # Velocities m/s (NED frame)
                            'groundspeed': math.sqrt(msg.vx**2 + msg.vy**2) / 100.0 # Calculate ground speed
                         })
                         if repr(drone_state) != orig_repr: state_updated = True # <-- ** FIX APPLIED HERE **

                elif msg_type == 'VFR_HUD':
                    with drone_state_lock:
                        orig_repr = repr(drone_state)
                        drone_state['airspeed'] = msg.airspeed # Airspeed in m/s
                        # Use groundspeed from VFR_HUD only if GLOBAL_POSITION_INT hasn't provided it yet
                        if drone_state.get('groundspeed', 0.0) == 0.0:
                            drone_state['groundspeed'] = msg.groundspeed
                        # drone_state['alt_rel'] = msg.alt # VFR_HUD altitude is often less precise? Stick with GLOBAL_POS_INT
                        # drone_state['climb'] = msg.climb # Climb rate m/s
                        if repr(drone_state) != orig_repr: state_updated = True

                elif msg_type == 'GPS_RAW_INT':
                    with drone_state_lock:
                        orig_repr = repr(drone_state)
                        drone_state.update({
                            'gps_fix_type': msg.fix_type,
                            'satellites_visible': msg.satellites_visible if msg.satellites_visible != 255 else -1, # 255 = invalid
                            'hdop': msg.eph / 100.0 if msg.eph != 65535 else 99.99 # GPS HDOP * 100, 65535 = invalid
                        })
                        if repr(drone_state) != orig_repr: state_updated = True

                elif msg_type == 'SYS_STATUS':
                    with drone_state_lock:
                        orig_repr = repr(drone_state)
                        flags = msg.onboard_control_sensors_health # EKF/Health flags
                        drone_state.update({
                            'battery_voltage': msg.voltage_battery / 1000.0 if msg.voltage_battery > 0 else 0.0, # Voltage in V
                            'battery_remaining': msg.battery_remaining if msg.battery_remaining >= 0 else -1, # Remaining percentage (-1 if invalid)
                            'battery_current': msg.current_battery / 100.0 if msg.current_battery >= 0 else -1.0, # Current in A (-1 if invalid)
                            'ekf_flags': flags,
                            'ekf_status_report': get_ekf_status_report(flags) # Generate human-readable EKF status
                        })
                        if repr(drone_state) != orig_repr: state_updated = True

                elif msg_type == 'STATUSTEXT':
                     # Handle status text messages from the drone
                     text = msg.text.rstrip('\x00') # Remove null terminators
                     severity = msg.severity
                     # Map MAV_SEVERITY levels to UI message types (e.g., 'info', 'warning', 'error')
                     sev_map = {
                         mavutil.mavlink.MAV_SEVERITY_EMERGENCY: 'error', mavutil.mavlink.MAV_SEVERITY_ALERT: 'error',
                         mavutil.mavlink.MAV_SEVERITY_CRITICAL: 'error', mavutil.mavlink.MAV_SEVERITY_ERROR: 'error',
                         mavutil.mavlink.MAV_SEVERITY_WARNING: 'warning', mavutil.mavlink.MAV_SEVERITY_NOTICE: 'info',
                         mavutil.mavlink.MAV_SEVERITY_INFO: 'info', mavutil.mavlink.MAV_SEVERITY_DEBUG: 'debug'
                     }
                     ui_type = sev_map.get(severity, 'info')
                     print(f"STATUSTEXT [{ui_type.upper()}]: {text}")
                     # Don't flood UI with debug messages
                     if ui_type != 'debug':
                         try:
                             socketio.emit('status_message', {'text': text, 'type': ui_type}, namespace='/drone')
                         except Exception as e:
                             print(f"Error emitting status message: {e}")

                elif msg_type == 'ATTITUDE':
                    with drone_state_lock:
                        orig_repr = repr(drone_state)
                        drone_state.update({'pitch': msg.pitch, 'roll': msg.roll}) # Radians
                        if repr(drone_state) != orig_repr: state_updated = True

                elif msg_type == 'HOME_POSITION':
                     # Received home position - store it and stop requesting
                     if not home_position_requested: # Process only the first one received
                         lat, lon = msg.latitude / 1e7, msg.longitude / 1e7
                         print(f"HOME_POSITION Received: Lat={lat:.7f}, Lon={lon:.7f}")
                         with drone_state_lock:
                             drone_state.update({'home_lat': lat, 'home_lon': lon})
                             state_updated = True # State changed
                         home_position_requested = True
                         # Stop requesting HOME_POSITION stream
                         request_data_streams()
                     # else: ignore subsequent HOME_POSITION messages

                elif msg_type == 'COMMAND_ACK':
                     # Handle acknowledgments for sent commands
                     cmd, res = msg.command, msg.result
                     res_txt = MAV_RESULT_STR.get(res, f"Unk({res})")
                     pending_cmd_info = pending_commands.pop(cmd, None) # Remove from pending list
                     name = pending_cmd_info['name'] if pending_cmd_info else (mavutil.mavlink.enums['MAV_CMD'][cmd].name if cmd in mavutil.mavlink.enums['MAV_CMD'] else f'ID {cmd}')
                     print(f"ACK Received: CMD={name}({cmd}), Result={res_txt}({res})")
                     try:
                         socketio.emit('command_ack_received', {
                             'command_id': cmd, 'command_name': name,
                             'result': res, 'result_text': res_txt
                         }, namespace='/drone')
                     except Exception as e:
                         print(f"Error emitting command ACK status: {e}")


                # --- Mission Download Handling (No drone_state_lock needed here, uses mission_state_lock) ---
                # Check if a mission download is active before processing mission messages
                # Ensure message is targeted to our system ID
                mission_msg_to_us = (hasattr(msg, 'target_system') and msg.target_system == mavlink_connection.source_system)

                if mission_msg_to_us and mission_download_state["active"]:
                    with mission_state_lock: # Lock mission state for updates
                        target_sys = mavlink_connection.target_system
                        target_comp = mavlink_connection.target_component
                        current_mission_type = mission_download_state["mission_type"]

                        if msg_type == 'MISSION_COUNT':
                            # Received the total number of mission items
                            # Ensure the type matches the type we requested
                            m_type_recv = getattr(msg, 'mission_type', current_mission_type) # Use state default if field missing
                            if m_type_recv == current_mission_type:
                                count = msg.count
                                print(f"Received MISSION_COUNT: {count} (Type: {m_type_recv})")
                                mission_download_state["expected_count"] = count
                                mission_download_state["received_count"] = 0
                                mission_download_state["waypoints"] = [] # Clear previous data
                                mission_download_state["fence_points"] = []

                                if count == 0:
                                    print("Mission/Fence list is empty.")
                                    # Send ACK immediately for empty list
                                    mavlink_connection.mav.mission_ack_send(target_sys, target_comp, MAV_MISSION_ACCEPTED, m_type_recv)
                                    socketio.emit('status_message', {'text': 'Mission/Fence list is empty.', 'type': 'info'}, namespace='/drone')
                                    reset_mission_download_state(emit_update=True) # Reset and notify UI
                                else:
                                    # Request the first item (index 0)
                                    print(f"Requesting first mission item (0 of {count})")
                                    mission_download_state["last_requested_index"] = 0
                                    mavlink_connection.mav.mission_request_int_send(target_sys, target_comp, 0, m_type_recv)
                                    schedule_mission_timeout() # Start/reset timeout waiting for item 0
                            else:
                                print(f"WARN: Received MISSION_COUNT for unexpected type {m_type_recv} (expected {current_mission_type}). Ignoring.")


                        elif msg_type == 'MISSION_ITEM_INT':
                            # Received a mission item
                            m_type_recv = getattr(msg, 'mission_type', current_mission_type)
                            seq_recv = msg.seq

                            # Check if sequence number is the one we expect and type matches
                            if seq_recv == mission_download_state["received_count"] and m_type_recv == current_mission_type:
                                print(f"Received MISSION_ITEM_INT: Seq={seq_recv}, Cmd={msg.command}, Frame={msg.frame}, Lat={msg.x/1e7}, Lon={msg.y/1e7}, Alt={msg.z}")
                                mission_download_state["received_count"] += 1

                                # Extract data - check for valid lat/lon (degrees * 1e7)
                                # Use None for invalid coordinates to prevent plotting errors
                                lat = msg.x / 1e7 if abs(msg.x) <= 90 * 1e7 else None
                                lon = msg.y / 1e7 if abs(msg.y) <= 180 * 1e7 else None
                                alt = msg.z # Altitude in meters (frame defines interpretation, e.g., MSL, AGL)

                                item_data = {
                                    "seq": seq_recv,
                                    "frame": msg.frame,
                                    "command": msg.command,
                                    "current": msg.current, # Boolean: is this the current WP?
                                    "autocontinue": msg.autocontinue, # Boolean
                                    "param1": msg.param1, "param2": msg.param2,
                                    "param3": msg.param3, "param4": msg.param4,
                                    "lat": lat, "lon": lon, "alt": alt,
                                    "mission_type": m_type_recv
                                }

                                # Categorize based on command ID and validity
                                if msg.command in FENCE_COMMANDS and lat is not None and lon is not None:
                                    mission_download_state["fence_points"].append(item_data)
                                # Consider NAV_WAYPOINT and potentially others as waypoints if coords valid
                                elif msg.command == MAV_CMD_NAV_WAYPOINT and lat is not None and lon is not None:
                                    mission_download_state["waypoints"].append(item_data)
                                # Add other mission commands that should be plotted if they have valid coords
                                # elif msg.command in [SOME_OTHER_NAV_CMD] and lat is not None and lon is not None:
                                #     mission_download_state["waypoints"].append(item_data)
                                else:
                                     # Item is not a plottable fence/waypoint or has invalid coords
                                     print(f"  -> Mission item {seq_recv} (Cmd: {msg.command}) has invalid coords or is not plottable. Storing data, skipping plot.")
                                     # Still store it if needed for full mission view/save later? For now, just store plottable ones.
                                     # if msg.command != 0: # Store non-empty commands even if not plottable?
                                     #    mission_download_state["waypoints"].append(item_data) # Or a separate list?


                                # Request next item or finalize
                                expected_count = mission_download_state["expected_count"]
                                received_count = mission_download_state["received_count"]
                                if received_count < expected_count:
                                    next_seq = received_count
                                    print(f"Requesting next mission item ({next_seq} of {expected_count})")
                                    mission_download_state["last_requested_index"] = next_seq
                                    mavlink_connection.mav.mission_request_int_send(target_sys, target_comp, next_seq, m_type_recv)
                                    schedule_mission_timeout() # Reset timeout for next item
                                else:
                                    # All items received
                                    print("Mission download complete. Sending final ACK.")
                                    mavlink_connection.mav.mission_ack_send(target_sys, target_comp, MAV_MISSION_ACCEPTED, m_type_recv)
                                    num_wp = len(mission_download_state["waypoints"])
                                    num_fp = len(mission_download_state["fence_points"])
                                    completion_msg = f'Mission/Fence download complete ({num_wp} WP, {num_fp} FP).'
                                    socketio.emit('status_message', {'text': completion_msg, 'type': 'info'}, namespace='/drone')
                                    # Send data to UI
                                    socketio.emit('mission_update', mission_download_state["waypoints"], namespace='/drone')
                                    socketio.emit('geofence_update', mission_download_state["fence_points"], namespace='/drone')
                                    reset_mission_download_state() # Keep data but reset flags/timeout

                            elif seq_recv != mission_download_state["received_count"] and m_type_recv == current_mission_type:
                                # Received item out of sequence
                                print(f"ERROR: Received mission item out of sequence: {seq_recv} (expected {mission_download_state['received_count']}). Aborting download.")
                                # Send NAK? Optional.
                                # mavlink_connection.mav.mission_ack_send(target_sys, target_comp, mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE, m_type_recv)
                                socketio.emit('status_message', {'text': f'Mission download failed: Incorrect sequence received ({seq_recv} vs {mission_download_state["received_count"]}).', 'type': 'error'}, namespace='/drone')
                                reset_mission_download_state(emit_update=True) # Reset and clear UI
                            # Silently ignore if mission_type doesn't match the active download


                        elif msg_type == 'MISSION_ACK':
                            # Handle ACKs related to mission operations (e.g., from upload later, or maybe error during download?)
                            m_type_recv = getattr(msg, 'mission_type', current_mission_type)
                            ack_type = msg.type # This is MAV_MISSION_RESULT enum value
                            ack_name = mavutil.mavlink.enums['MAV_MISSION_RESULT'][ack_type].name if ack_type in mavutil.mavlink.enums['MAV_MISSION_RESULT'] else f'UNK({ack_type})'
                            print(f"Received MISSION_ACK: Type={ack_name}({ack_type}), MissionType={m_type_recv}")

                            # If we receive an error ACK during an active download, abort it.
                            if ack_type != MAV_MISSION_ACCEPTED and m_type_recv == current_mission_type:
                                 print(f"ERROR: Mission download failed. Received NAK: {ack_name}")
                                 socketio.emit('status_message', {'text': f'Mission download failed: {ack_name}', 'type': 'error'}, namespace='/drone')
                                 reset_mission_download_state(emit_update=True) # Reset and clear UI

                # --- End Mission Handling ---

                # If telemetry state was updated by any handler, set the global flag
                # Note: the state_updated flag is scoped within the message processing block.
                # We must set the global drone_state_changed flag *outside* the lock
                # but *inside* the 'if state_updated:' check.
                if state_updated:
                    # This assignment happens AFTER the 'with drone_state_lock:' block for the specific message type
                    # but BEFORE the next message is processed or the loop yields.
                    drone_state_changed = True # Set the global flag

                # Brief yield after processing a message
                gevent.sleep(0.001)

            except Exception as parse_err:
                print(f"!! CRITICAL Error processing {msg_type} (SYSID:{source_sys}): {parse_err}")
                traceback.print_exc()
                # Potentially reset connection or state here if errors are frequent?
                # For now, just log and continue.
        else:
            # No message received in timeout, yield control slightly longer
            # This allows other greenlets (like telemetry update) to run
            gevent.sleep(0.005)


def periodic_telemetry_update():
    global drone_state_changed
    while True:
        try:
            emit_now = False
            state_copy = {} # Hold a copy of the state to emit
            with drone_state_lock:
                 # Check if the global state has changed since last emit
                 if drone_state_changed:
                     state_copy = drone_state.copy() # Shallow copy is usually sufficient
                     drone_state_changed = False # Reset global flag *after* copying
                     emit_now = True

            # Emit outside the lock to avoid holding it during network I/O
            if emit_now:
                # print(f"DBG: Emitting telemetry_update at {time.time()}") # Debug frequency
                socketio.emit('telemetry_update', state_copy, namespace='/drone')

        except Exception as e:
            print(f"Error in periodic_telemetry_update: {e}")
            traceback.print_exc() # Log errors but keep the loop running

        # Sleep for the desired interval
        gevent.sleep(TELEMETRY_UPDATE_INTERVAL)


# --- Flask Routes & SocketIO Handlers ---
@app.route('/')
def index():
    # Pass the script version to the template
    return render_template("index.html", version=SCRIPT_VERSION)

@app.route('/mavlink_dump')
def mavlink_dump():
     # Pass the script version to the template
    return render_template("mavlink_dump.html", version=SCRIPT_VERSION)

@socketio.on('connect', namespace='/drone')
def handle_connect():
    client_sid = request.sid
    print(f"Web UI Client connected: {client_sid}")
    # Send current state immediately on connection
    with drone_state_lock:
        emit('telemetry_update', drone_state.copy()) # Send current telemetry
    # Send current mission/fence state (might be empty)
    with mission_state_lock:
        emit('mission_update', mission_download_state["waypoints"])
        emit('geofence_update', mission_download_state["fence_points"])
    # Send connection status message
    conn_status = 'Drone link active.' if drone_state.get('connected') else 'Attempting drone link...'
    emit('status_message', {'text': f'Backend connected (v{SCRIPT_VERSION}). {conn_status}', 'type': 'info'})

@socketio.on('disconnect', namespace='/drone')
def handle_disconnect():
    client_sid = request.sid
    print(f"Web UI Client disconnected: {client_sid}")


# --- Command Sending Function ---
def send_mavlink_command(command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, confirmation=0):
    global mavlink_connection, pending_commands
    # Ensure connection exists and we have a target system ID
    if not mavlink_connection or not drone_state.get("connected", False) or mavlink_connection.target_system == 0:
        name = mavutil.mavlink.enums['MAV_CMD'][command].name if command in mavutil.mavlink.enums['MAV_CMD'] else f'ID {command}'
        err_msg = "Disconnected" if not drone_state.get("connected", False) else "Invalid Target System"
        msg = f"CMD {name} Failed: {err_msg}."
        print(f"ERROR: {msg}")
        # Emit failure to UI? Yes, consistent feedback is good.
        socketio.emit('status_message', {'text': msg, 'type': 'error'}, namespace='/drone')
        socketio.emit('command_result', {'command_id': command, 'command_name': name, 'success': False, 'message': msg}, namespace='/drone')
        return (False, msg)

    target_sys = mavlink_connection.target_system
    target_comp = mavlink_connection.target_component # Use the component ID from heartbeat

    try:
        # Get command name for logging/feedback
        name = mavutil.mavlink.enums['MAV_CMD'][command].name if command in mavutil.mavlink.enums['MAV_CMD'] else f'ID {command}'
        print(f"Sending CMD {name}({command}) to SYS:{target_sys} COMP:{target_comp} | P: {p1:.2f},{p2:.2f},{p3:.2f},{p4:.2f},{p5:.6f},{p6:.6f},{p7:.2f} | Confirm:{confirmation}")

        # Send COMMAND_LONG
        mavlink_connection.mav.command_long_send(
            target_sys, target_comp, command,
            confirmation, # Confirmation counter (0 for first attempt)
            p1, p2, p3, p4, p5, p6, p7
        )

        # Add to pending commands dict IF an ACK is expected
        # Avoid adding commands that don't typically send ACKs (like interval setting)
        # Or commands we don't need to track ACK for (like requesting messages)
        if command not in [mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE]:
            # Store command details for better ACK reporting and timeout handling
            # Use command ID as key. If multiple sent quickly, last one overwrites, but ACK should match ID.
            pending_commands[command] = {'time': time.time(), 'name': name}
            # Limit queue size to prevent memory leak if ACKs are missed
            if len(pending_commands) > 30:
                # Remove the oldest entry (OrderedDict preserves insertion order)
                oldest_cmd_id = next(iter(pending_commands))
                oldest_name = pending_commands[oldest_cmd_id]['name']
                print(f"WARN: Pending cmd queue limit reached, removing oldest unacknowledged cmd: {oldest_name} ({oldest_cmd_id})")
                del pending_commands[oldest_cmd_id]

        return (True, f"CMD {name} sent.")
    except Exception as e:
        name = mavutil.mavlink.enums['MAV_CMD'][command].name if command in mavutil.mavlink.enums['MAV_CMD'] else f'ID {command}'
        msg = f"CMD {name} Send Error: {e}"
        print(f"ERROR: {msg}")
        traceback.print_exc()
        # Emit failure to UI
        socketio.emit('status_message', {'text': msg, 'type': 'error'}, namespace='/drone')
        socketio.emit('command_result', {'command_id': command, 'command_name': name, 'success': False, 'message': msg}, namespace='/drone')
        return (False, msg)

# --- UI Command Handler ---
@socketio.on('send_command', namespace='/drone')
def handle_send_command(data):
    cmd = data.get('command') # e.g., 'ARM', 'TAKEOFF'
    print(f"UI Command Received: {cmd} | Data: {data}")

    # Basic check for connection before proceeding
    if not drone_state.get("connected", False):
        msg = f'CMD {cmd} Fail: Drone Disconnected.'
        socketio.emit('status_message', {'text': msg, 'type': 'error'}, namespace='/drone')
        socketio.emit('command_result', {'command': cmd, 'success': False, 'message': msg}, namespace='/drone')
        print(f"WARN: {msg}")
        return

    success = False; msg = f'{cmd} processing...'; type = 'info' # Default values

    # --- Handle Specific Commands ---
    if cmd == 'ARM':
        # Param 1: 1 to arm, 0 to disarm
        success, msg_send = send_mavlink_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1)
        type = 'info' if success else 'error'
        msg = f'Arm command sent.' if success else f'Arm Fail: {msg_send}'
    elif cmd == 'DISARM':
        # Param 1: 0 to disarm
        success, msg_send = send_mavlink_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=0)
        type = 'info' if success else 'error'
        msg = f'Disarm command sent.' if success else f'Disarm Fail: {msg_send}'
    elif cmd == 'TAKEOFF':
        try:
            alt = float(data.get('altitude', 5.0)) # Default 5m if not provided
            if not (0 < alt <= 1000): # Basic validation
                raise ValueError("Altitude must be between 0 and 1000m")
            # Param 7: Altitude in meters
            # ArduPilot usually requires GUIDED or AUTO mode for takeoff cmd
            # Consider adding pre-flight checks here later (mode, GPS fix, etc.)
            success, msg_send = send_mavlink_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=alt)
            type = 'info' if success else 'error'
            msg = f'Takeoff to {alt:.1f}m command sent.' if success else f'Takeoff Fail: {msg_send}'
        except (ValueError, TypeError) as e:
            success = False; type = 'error'; msg = f'Invalid Takeoff Altitude: {e}'
    elif cmd == 'LAND':
        # No specific parameters needed usually
        success, msg_send = send_mavlink_command(mavutil.mavlink.MAV_CMD_NAV_LAND)
        type = 'info' if success else 'error'
        msg = f'Land command sent.' if success else f'Land Fail: {msg_send}'
    elif cmd == 'RTL':
        # No specific parameters needed usually
        success, msg_send = send_mavlink_command(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        type = 'info' if success else 'error'
        msg = f'RTL command sent.' if success else f'RTL Fail: {msg_send}'
    elif cmd == 'SET_MODE':
        mode_str = data.get('mode_string') # e.g., "GUIDED", "LOITER"
        if mode_str and mode_str in AP_CUSTOM_MODES:
            custom_mode_id = AP_CUSTOM_MODES[mode_str]
            # MAV_CMD_DO_SET_MODE uses:
            # p1: MAV_MODE flags (use CUSTOM_MODE_ENABLED)
            # p2: Custom mode number (ArduPilot Copter/Rover primary)
            # p3: Custom mode number (ArduPilot Plane primary) - Use 0 if not plane? Or send same ID? Let's use p2.
            base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            print(f"Attempting to set mode: {mode_str} (ID: {custom_mode_id})")
            success, msg_send = send_mavlink_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, p1=base_mode, p2=custom_mode_id)
            type = 'info' if success else 'error'
            msg = f'Set Mode {mode_str} command sent.' if success else f'Set Mode Fail: {msg_send}'
        else:
            success = False; type = 'error'; msg = f'Invalid or unknown mode string: {mode_str}'
    elif cmd == 'GOTO':
        try:
            lat = float(data['lat'])
            lon = float(data['lon'])
            alt = float(data['alt']) # Assume AGL for now based on UI placeholder
            # Basic validation
            if not (-90 <= lat <= 90): raise ValueError("Latitude out of range [-90, 90]")
            if not (-180 <= lon <= 180): raise ValueError("Longitude out of range [-180, 180]")
            if not (-100 <= alt <= 5000): raise ValueError("Altitude out of range [-100, 5000]") # Reasonable limits?

            # MAV_CMD_DO_REPOSITION:
            # p1: Target ground speed (m/s), -1 for default/no change.
            # p2: Bitmask for behavior flags (ignore for basic GoTo)
            # p3: Loiter radius (m), 0 for default.
            # p4: Yaw heading (degrees), NaN to ignore.
            # p5: Latitude (degrees)
            # p6: Longitude (degrees)
            # p7: Altitude (meters). Frame depends on FC config, often relative to home/takeoff for GUIDED.
            success, msg_send = send_mavlink_command(
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                p1=-1,         # Use default ground speed
                p4=math.nan,   # Don't change yaw heading
                p5=lat,
                p6=lon,
                p7=alt
            )
            type = 'info' if success else 'error'
            msg = f'GoTo command sent ({lat:.6f},{lon:.6f},{alt:.1f}m).' if success else f'GoTo Fail: {msg_send}'
        except (ValueError, TypeError, KeyError) as e:
            success = False; type = 'error'; msg = f'Invalid GoTo parameters: {e}'
    else:
        # Unknown command from UI
        success = False; type = 'warning'; msg = f'Unknown command received: {cmd}'

    # --- Emit feedback to UI ---
    socketio.emit('status_message', {'text': msg, 'type': type}, namespace='/drone')
    # Emit command_result for potential UI state updates (e.g., re-enabling button on failure)
    socketio.emit('command_result', {'command': cmd, 'success': success, 'message': msg}, namespace='/drone')


# --- Mission Download Request Handler ---
@socketio.on('request_mission_download', namespace='/drone')
def handle_request_mission_download(data):
    global mavlink_connection
    # Determine mission type to request (default to standard mission)
    # ArduPilot uses MAV_MISSION_TYPE_MISSION for waypoints and MAV_MISSION_TYPE_FENCE for fence.
    # Let's request MISSION first. If needed, add logic to request FENCE separately.
    # For simplicity now, just request MISSION, assuming fence points might be included there or
    # we add a separate button later. Let's assume data can specify type, default MISSION.
    mission_type_to_req = data.get("mission_type", MAV_MISSION_TYPE_MISSION)
    type_name = "Mission" if mission_type_to_req == MAV_MISSION_TYPE_MISSION else "Fence" if mission_type_to_req == MAV_MISSION_TYPE_FENCE else f"Type {mission_type_to_req}"

    print(f"UI requested {type_name} download.")

    # Check connection
    if not mavlink_connection or not drone_state.get("connected", False):
        emit('status_message', {'text': f'Cannot download {type_name}: Disconnected.', 'type': 'error'})
        print(f"WARN: Cannot download {type_name}, disconnected.")
        return

    target_sys = mavlink_connection.target_system
    target_comp = mavlink_connection.target_component

    # Check if another download is already in progress
    with mission_state_lock:
        if mission_download_state["active"]:
            emit('status_message', {'text': 'Mission/Fence download already in progress.', 'type': 'warning'})
            print("WARN: Download requested while another is active.")
            return

        # Reset state before starting new download
        print(f"Initiating {type_name} download process...")
        reset_mission_download_state(emit_update=True) # Clear old data in UI too

        # Set state for the new download
        mission_download_state["active"] = True
        mission_download_state["start_time"] = time.time()
        mission_download_state["mission_type"] = mission_type_to_req # Store the type we are requesting

    # Send the request list message
    try:
        # Use MISSION_REQUEST_LIST message (ID 43)
        mavlink_connection.mav.mission_request_list_send(target_sys, target_comp, mission_type_to_req)
        print(f"Sent MISSION_REQUEST_LIST (Type: {mission_type_to_req})")
        emit('status_message', {'text': f'Requesting {type_name} list...', 'type': 'info'})
        # Start the timeout waiting for the MISSION_COUNT response
        schedule_mission_timeout()
    except Exception as e:
        print(f"Error sending MISSION_REQUEST_LIST: {e}")
        traceback.print_exc()
        emit('status_message', {'text': f'Error sending {type_name} request: {e}', 'type': 'error'})
        reset_mission_download_state() # Reset state on send failure


# --- Main Execution ---
if __name__ == '__main__':
    print(f"--- Starting Drone Control Backend v{SCRIPT_VERSION} ---")
    # Start the MAVLink listening greenlet
    print("Spawning MAVLink listener greenlet...")
    mavlink_thread = gevent.spawn(mavlink_receive_loop)

    # Start the periodic telemetry update greenlet
    print("Spawning telemetry update greenlet...")
    telemetry_update_thread = gevent.spawn(periodic_telemetry_update)

    # Start the Flask-SocketIO web server using Gevent's WSGI server
    print(f"Starting Flask-SocketIO web server on http://{WEB_SERVER_HOST}:{WEB_SERVER_PORT}")
    print(f"Access UI (v{SCRIPT_VERSION}) at http://<raspberry_pi_ip>:{WEB_SERVER_PORT} or http://127.0.0.1:{WEB_SERVER_PORT}")
    server = None # Initialize server variable
    try:
        # Create the WSGI server instance
        server = pywsgi.WSGIServer((WEB_SERVER_HOST, WEB_SERVER_PORT), app, handler_class=WebSocketHandler)
        print("Gevent WSGIServer created. Starting server.serve_forever()...")
        # Start serving requests indefinitely
        server.serve_forever()
    except KeyboardInterrupt:
        # Handle graceful shutdown on Ctrl+C
        print("\nCtrl+C received. Shutting down gracefully...")
    except OSError as e:
        # Handle common errors like port already in use
        print(f"ERROR starting web server: {e}")
        if "Address already in use" in str(e):
            print("Please ensure another instance is not running or wait for the port to become free.")
        reset_mission_download_state() # Clean up mission state if server fails to start
        sys.exit(1) # Exit with error code
    except Exception as e:
        # Handle any other unexpected errors during server startup or runtime
        print(f"Unhandled Web Server Error: {e}")
        traceback.print_exc()
        reset_mission_download_state() # Clean up mission state on generic error
        sys.exit(1) # Exit with error code
    finally:
        # This block executes on normal exit or exception
        print("--- Server shutting down ---");
        reset_mission_download_state() # Ensure mission state is cleaned up before exit

        # Stop the Gevent WSGI server
        if server and hasattr(server, 'stop'):
            print("Stopping Gevent server...")
            server.stop(timeout=2) # Allow 2 seconds for active connections to close

        # Stop the background greenlets
        print("Stopping background greenlets...")
        if telemetry_update_thread and not telemetry_update_thread.dead:
            print("Killing telemetry update greenlet...")
            telemetry_update_thread.kill(block=False) # Don't block excessively
        if mavlink_thread and not mavlink_thread.dead:
            print("Killing MAVLink listener greenlet...")
            mavlink_thread.kill(block=True, timeout=2) # Allow time for potential cleanup

        # Close the MAVLink connection if it's still open
        if mavlink_connection:
             try:
                  print("Closing MAVLink connection...")
                  mavlink_connection.close()
                  print("MAVLink connection closed.")
             except Exception as e:
                  print(f"Error closing MAVLink connection during shutdown: {e}")

        print("--- Shutdown complete ---")
EOF
# Set ownership for the created file
sudo chown ${INSTALL_USER}:${INSTALL_GROUP} "${APP_DIR}/app.py"


print_info "Creating ${APP_DIR}/templates/index.html (v${SCRIPT_VERSION})..."
# Use temporary variable approach for complex HTML with potential special chars
# Ensure {{ version }} is replaced with ${SCRIPT_VERSION} for bash expansion
INDEX_HTML_CONTENT=$(cat <<EOF
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Drone Control Interface v${SCRIPT_VERSION}</title> <!-- Use script var -->
    <link rel="stylesheet" href="/static/lib/leaflet.css"/>
    <link rel="stylesheet" href="/static/css/style.css"/>
    <style>
        /* Styles from v5.11 base */
        body{display:flex;flex-direction:column;height:100vh;margin:0}
        h1.main-title{background-color:#333;color:#fff;padding:8px 15px;margin:0;font-size:1.2em;flex-shrink:0}
        .main-container{display:flex;flex:1;overflow:hidden}
        button:disabled,select:disabled{background-color:#ccc;color:#666;cursor:not-allowed;opacity:.7}
        .map-controls{position:absolute;top:10px;left:50px;z-index:500;background:rgba(255,255,255,.8);padding:5px;border-radius:4px;border:1px solid #bbb}
        .map-controls button{font-size:11px;padding:3px 6px;margin-left:5px; background-color:#fff; color:#333; border: 1px solid #aaa;} /* Added base style */
        .map-controls button:hover:not(:disabled){background-color:#eee;} /* Added base style */
        .goto-controls{display:flex;align-items:center;flex-wrap:wrap;gap:4px}
        .goto-controls input{flex-grow:1;min-width:60px}
        .mode-select-control{margin-left:10px}
        .mode-select-control select{padding:5px;font-size:.9em}
        /* Styles added in v5.12 */
        .mission-controls { /* Style for the new button section */
            margin-top: 10px;
            padding-top: 10px;
            border-top: 1px solid #bbb;
        }
        .mission-controls button {
             background-color: #ffc107; /* Orange color for mission button */
             color: #333;
             margin-right: 10px;
        }
         .mission-controls button:hover:not(:disabled) {
             background-color: #e0a800;
        }
        /* Style for waypoint markers */
        .waypoint-div-icon {
            background: #007bff; /* Blue background */
            color: white;       /* White text */
            border-radius: 50%; /* Circular shape */
            text-align: center;
            line-height: 18px;  /* Center text vertically */
            font-size: 10px;
            font-weight: bold;
            border: 1px solid white; /* White border */
            box-shadow: 1px 1px 3px rgba(0,0,0,0.5); /* Slight shadow */
            width: 18px;        /* Fixed size */
            height: 18px;       /* Fixed size */
        }
    </style>
</head>
<body>
    <h1 class="main-title">Drone Control Interface v${SCRIPT_VERSION}</h1> <!-- Use script var -->
    <div class="main-container">
        <div class="left-column">
            <div class="pfd-section">
                <!-- PFD canvas and overlays (identical to v5.12) -->
                <div id="attitude-indicator-wrapper">
                    <canvas id="attitude-canvas" width="280" height="250"></canvas>
                    <div id="asi-tape-container"><canvas id="asi-canvas" width="60" height="250"></canvas></div>
                    <div id="alt-tape-container"><canvas id="alt-canvas" width="70" height="250"></canvas></div>
                    <div id="arming-status-overlay" class="pfd-overlay">DISARMED</div>
                    <div id="battery-overlay" class="pfd-overlay">Bat: <span id="battery-value">--.-</span>V</div>
                    <div id="mode-overlay" class="pfd-overlay">Mode: <span id="mode-value">------</span></div>
                    <div id="current-overlay" class="pfd-overlay">Cur: <span id="current-value">--.-</span>A</div>
                    <div id="gps-overlay" class="pfd-overlay">GPS: <span id="gps-fix-value">---</span> (<span id="gps-sats-value">0</span>) HDOP:<span id="gps-hdop-value">--.-</span></div>
                    <div id="latlon-overlay" class="pfd-overlay">Lat: <span id="lat-value">--.------</span><br>Lon: <span id="lon-value">--.------</span></div>
                </div>
            </div>
            <div class="info-controls-section">
                <div class="status">
                    <h2>Status</h2>
                    <!-- Status grid (identical to v5.12) -->
                    <div class="status-grid">
                        <div class="status-item"><strong>Connection:</strong> <span id="status-connection" class="disconnected">Disconnected</span><span id="heartbeat-indicator" class="heartbeat-icon" title="Heartbeat Status">❤️</span></div>
                        <div class="status-item"><strong>EKF Status:</strong> <span id="ekf-status" class="ekf-status ekf-error">INIT</span></div>
                        <div id="messages" class="message-log"><strong>Messages:</strong><br></div>
                    </div>
                </div>
                <div class="controls">
                    <h2>Controls</h2>
                    <!-- Arm/Disarm/Takeoff/Land/RTL buttons (identical to v5.12) -->
                    <div>
                        <button id="btn-arm">Arm</button> <button id="btn-disarm">Disarm</button>
                        <button id="btn-takeoff">Takeoff</button> (Alt: <input type="number" id="takeoff-alt" value="5" step="1" min="1" max="1000" style="width:45px"> m)
                        <button id="btn-land">Land</button> <button id="btn-rtl">RTL</button>
                    </div>
                    <!-- Mode select (identical to v5.12) -->
                    <div class="mode-select-control">
                        <label for="mode-select">Set Mode:</label>
                        <select id="mode-select">
                            <option value="">--Select Mode--</option><option value="STABILIZE">Stabilize</option><option value="ALT_HOLD">Alt Hold</option><option value="POS_HOLD">Pos Hold</option><option value="LOITER">Loiter</option><option value="GUIDED">Guided</option><option value="RTL">RTL</option><option value="LAND">Land</option><option value="AUTO">Auto</option><option value="BRAKE">Brake</option>
                        </select>
                        <button id="btn-set-mode">Set</button>
                    </div>
                    <hr>
                    <!-- GoTo controls (identical to v5.12) -->
                     <div class="goto-controls"> GoTo: Lat <input type="number" step="any" id="goto-lat" placeholder="Lat [-90,90]"> Lon <input type="number" step="any" id="goto-lon" placeholder="Lon [-180,180]"> Alt <input type="number" step="any" id="goto-alt" placeholder="Alt (AGL)" value="10" min="-100" max="5000"> <button id="btn-goto">Go To</button> <button id="btn-clear-goto" title="Clear GoTo Fields">Clear</button> </div>
                    <!-- Mission Controls (identical to v5.12) -->
                    <div class="mission-controls">
                         <button id="btn-load-mission">Load Mission/Fence</button>
                         <!-- Add Clear button later if needed: <button id="btn-clear-mission">Clear Map</button> -->
                    </div>
                </div>
            </div>
        </div>
        <div class="right-column" id="map">
            <div class="map-controls">
                <button id="btn-center-map" title="Center map on drone">Center Drone</button>
                <button id="btn-fit-mission" title="Zoom to fit mission items">Fit Mission</button> <!-- Identical to v5.12 -->
            </div>
        </div>
    </div>

    <a href="/mavlink_dump" id="dump-link-button" target="_blank" title="Open MAVLink Message Dump">Dump</a>

    <script src="/static/lib/leaflet.js"></script>
    <script src="/static/lib/socket.io.min.js"></script>
    <script>
        // --- Constants and Setup (Leaflet, SocketIO, PFD vars, Layers) ---
        // (Identical JS setup to v5.12)
        const map = L.map('map').setView([0, 0], 2);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '© OpenStreetMap contributors' }).addTo(map);
        let droneMarker = null, homeMarker = null, firstLocationUpdate = true;
        const droneIcon = L.icon({ iconUrl: 'data:image/svg+xml;base64,'+btoa('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="#007bff"><path d="M21 16v-2l-8-5V3.5c0-.83-.67-1.5-1.5-1.5S10 2.67 10 3.5V9l-8 5v2l8-2.5V19l-2 1.5V22l3.5-1 3.5 1v-1.5L13 19v-5.5l8 2.5z"/></svg>'), iconSize: [24, 24], iconAnchor: [12, 12] });
        const homeIcon = L.icon({ iconUrl: 'data:image/svg+xml;base64,'+btoa('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="#28a745"><path d="M10 20v-6h4v6h5v-8h3L12 3 2 12h3v8h5z"/></svg>'), iconSize: [24, 24], iconAnchor: [12, 24] });
        let waypointLayerGroup = L.layerGroup().addTo(map);
        let geofenceLayerGroup = L.layerGroup().addTo(map);
        let missionPolyline = null;
        const socket = io.connect(location.protocol + '//' + document.domain + ':' + location.port + '/drone', { reconnectionAttempts: 5, reconnectionDelay: 3000 });
        const messagesDiv = document.getElementById('messages');
        const ahCanvas = document.getElementById('attitude-canvas'), ahCtx = ahCanvas.getContext('2d');
        const asiCanvas = document.getElementById('asi-canvas'), asiCtx = asiCanvas.getContext('2d');
        const altCanvas = document.getElementById('alt-canvas'), altCtx = altCanvas.getContext('2d');
        const armingStatusOverlay = document.getElementById('arming-status-overlay');
        const batteryOverlay = document.getElementById('battery-overlay'); const modeOverlay = document.getElementById('mode-overlay'); const currentOverlay = document.getElementById('current-overlay'); const gpsOverlay = document.getElementById('gps-overlay'); const latlonOverlay = document.getElementById('latlon-overlay'); const batteryValueSpan = document.getElementById('battery-value'); const modeValueSpan = document.getElementById('mode-value'); const currentValueSpan = document.getElementById('current-value'); const gpsFixValueSpan = document.getElementById('gps-fix-value'); const gpsSatsValueSpan = document.getElementById('gps-sats-value'); const gpsHdopValueSpan = document.getElementById('gps-hdop-value'); const latValueSpan = document.getElementById('lat-value'); const lonValueSpan = document.getElementById('lon-value'); const ekfStatusSpan = document.getElementById('ekf-status');
        const commandControls = { ARM: document.getElementById("btn-arm"), DISARM: document.getElementById("btn-disarm"), TAKEOFF: document.getElementById("btn-takeoff"), LAND: document.getElementById("btn-land"), RTL: document.getElementById("btn-rtl"), SET_MODE: document.getElementById("btn-set-mode"), GOTO: document.getElementById("btn-goto"), "clear-goto": document.getElementById("btn-clear-goto"), "center-map": document.getElementById("btn-center-map"), LOAD_MISSION: document.getElementById("btn-load-mission"), FIT_MISSION: document.getElementById("btn-fit-mission") };
        const modeSelect = document.getElementById("mode-select");
        const takeoffAltInput = document.getElementById("takeoff-alt");

        // --- PFD Drawing Functions (Identical to v5.12) ---
        const DEG_TO_RAD = Math.PI / 180; const RAD_TO_DEG = 180 / Math.PI; const M_S_TO_KNOTS = 1.94384;
        const ahWidth = ahCanvas.width, ahHeight = ahCanvas.height; const ahCenterX = ahWidth / 2, ahCenterY = ahHeight / 2; const PITCH_SCALE_FACTOR = 4; const TAPE_WIDTH = asiCanvas.width; const TAPE_ALT_WIDTH = altCanvas.width; const TAPE_HEIGHT = ahHeight; const TAPE_CENTER_Y = TAPE_HEIGHT / 2; const ASI_PIXELS_PER_UNIT = 5; const ALT_PIXELS_PER_UNIT = 5; const HEADING_PIXELS_PER_DEG = 2.5; const HEADING_TAPE_HEIGHT = 25; const PFD_SKY_COLOR = "#87CEEB"; const PFD_GROUND_COLOR = "#A0522D"; const PFD_HORIZON_LINE_COLOR = "#FFFFFF"; const PFD_AIRCRAFT_SYMBOL_COLOR = "#FFFF00"; const PFD_TAPE_TICK_COLOR = "#FFFFFF"; const PFD_TAPE_POINTER_COLOR = "#FFFF00"; const PFD_OVERLAY_BOX_COLOR = "rgba(50, 50, 50, 0.7)"; const PFD_OVERLAY_TEXT_COLOR = "#FFFFFF";
        function drawRoundedRect(ctx, x, y, width, height, radius) { ctx.beginPath(); ctx.moveTo(x + radius, y); ctx.lineTo(x + width - radius, y); ctx.arcTo(x + width, y, x + width, y + radius, radius); ctx.lineTo(x + width, y + height - radius); ctx.arcTo(x + width, y + height, x + width - radius, y + height, radius); ctx.lineTo(x + radius, y + height); ctx.arcTo(x, y + height, x, y + height - radius, radius); ctx.lineTo(x, y + radius); ctx.arcTo(x, y, x + radius, y, radius); ctx.closePath(); }
        function drawAttitudeIndicator(pitch = 0, roll = 0, heading = 0) { const rollRad = -roll * DEG_TO_RAD; const pitchPx = pitch * PITCH_SCALE_FACTOR; const ctx = ahCtx; ctx.save(); ctx.fillStyle = "#000"; ctx.fillRect(0, 0, ahWidth, ahHeight); ctx.fillStyle = "rgba(0, 0, 0, 0.6)"; ctx.fillRect(0, 0, ahWidth, HEADING_TAPE_HEIGHT); ctx.save(); ctx.strokeStyle = PFD_TAPE_TICK_COLOR; ctx.fillStyle = PFD_TAPE_TICK_COLOR; ctx.font = "bold 12px monospace"; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.beginPath(); ctx.rect(0, 0, ahWidth, HEADING_TAPE_HEIGHT); ctx.clip(); ctx.translate(ahCenterX - heading * HEADING_PIXELS_PER_DEG, HEADING_TAPE_HEIGHT / 2); const headingRange = Math.ceil(ahWidth / (2 * HEADING_PIXELS_PER_DEG)); const startHdg = Math.floor(heading - headingRange - 10); const endHdg = Math.ceil(heading + headingRange + 10); const headingLabels = { 0: "N", 30: "3", 60: "6", 90: "E", 120: "12", 150: "15", 180: "S", 210: "21", 240: "24", 270: "W", 300: "30", 330: "33", 360: "N" }; for (let h = startHdg; h <= endHdg; h += 5) { const normalizedHdg = (h % 360 + 360) % 360; const xPos = h * HEADING_PIXELS_PER_DEG; let tickHeight = 5; let isLabelTick = false; let label = null; ctx.lineWidth = 1; if (h % 10 === 0) tickHeight = 10; if (h % 30 === 0) { tickHeight = 15; isLabelTick = true; label = (normalizedHdg / 10).toString(); } if (headingLabels[normalizedHdg]) { tickHeight = 15; isLabelTick = true; label = headingLabels[normalizedHdg]; ctx.lineWidth = 1.5; } ctx.beginPath(); ctx.moveTo(xPos, -tickHeight / 2); ctx.lineTo(xPos, tickHeight / 2); ctx.stroke(); if (isLabelTick && label) { ctx.fillText(label, xPos, tickHeight / 2 + 10); } } ctx.restore(); ctx.save(); ctx.beginPath(); ctx.rect(0, HEADING_TAPE_HEIGHT, ahWidth, ahHeight - HEADING_TAPE_HEIGHT); ctx.clip(); ctx.translate(ahCenterX, ahCenterY + HEADING_TAPE_HEIGHT / 2); ctx.rotate(rollRad); ctx.translate(0, pitchPx); ctx.fillStyle = PFD_SKY_COLOR; ctx.fillRect(-ahWidth * 1.5, -ahHeight * 2, ahWidth * 3, ahHeight * 2); ctx.fillStyle = PFD_GROUND_COLOR; ctx.fillRect(-ahWidth * 1.5, 0, ahWidth * 3, ahHeight * 2); ctx.strokeStyle = PFD_HORIZON_LINE_COLOR; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(-ahWidth * 1.5, 0); ctx.lineTo(ahWidth * 1.5, 0); ctx.stroke(); ctx.strokeStyle = PFD_HORIZON_LINE_COLOR; ctx.lineWidth = 1; ctx.font = "12px monospace"; ctx.fillStyle = PFD_HORIZON_LINE_COLOR; ctx.textAlign = "center"; const tickLengthShort = 25; const tickLengthLong = 50; for (let p = -90; p <= 90; p += 5) { if (p === 0) continue; const yPos = -p * PITCH_SCALE_FACTOR; if (Math.abs(yPos) > ahHeight * 1.5) continue; const isMajorTick = (p % 10 === 0); const tickLength = isMajorTick ? tickLengthLong : tickLengthShort; ctx.beginPath(); ctx.moveTo(-tickLength / 2, yPos); ctx.lineTo(tickLength / 2, yPos); ctx.stroke(); if (isMajorTick) { ctx.fillText(Math.abs(p).toString(), -tickLength / 2 - 20, yPos + 4); ctx.fillText(Math.abs(p).toString(), tickLength / 2 + 20, yPos + 4); } } ctx.restore(); ctx.save(); ctx.translate(ahCenterX, ahCenterY + HEADING_TAPE_HEIGHT / 2); ctx.strokeStyle = PFD_AIRCRAFT_SYMBOL_COLOR; ctx.fillStyle = PFD_AIRCRAFT_SYMBOL_COLOR; ctx.lineWidth = 2; const notchSize = 4; ctx.beginPath(); ctx.moveTo(0, -notchSize / 2); ctx.lineTo(-notchSize, notchSize / 2); ctx.lineTo(notchSize, notchSize / 2); ctx.closePath(); ctx.fillRect(-1,-1,2,2); const wingLength = 40; ctx.beginPath(); ctx.moveTo(-wingLength, 0); ctx.lineTo(-notchSize*1.5, 0); ctx.moveTo(notchSize*1.5, 0); ctx.lineTo(wingLength, 0); ctx.stroke(); ctx.restore(); ctx.save(); const hdgBoxWidth = 80; const hdgBoxHeight = 20; const hdgBoxX = ahCenterX - hdgBoxWidth / 2; const hdgBoxY = 5; const hdgBoxRadius = 5; ctx.fillStyle = PFD_OVERLAY_BOX_COLOR; drawRoundedRect(ctx, hdgBoxX, hdgBoxY, hdgBoxWidth, hdgBoxHeight, hdgBoxRadius); ctx.fill(); ctx.fillStyle = PFD_OVERLAY_TEXT_COLOR; ctx.font = "bold 11px monospace"; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(\`HDG: \${heading.toFixed(0)}°\`, ahCenterX, hdgBoxY + hdgBoxHeight / 2); ctx.restore(); ctx.fillStyle = PFD_TAPE_POINTER_COLOR; ctx.beginPath(); ctx.moveTo(ahCenterX, HEADING_TAPE_HEIGHT); ctx.lineTo(ahCenterX - 6, HEADING_TAPE_HEIGHT + 6); ctx.lineTo(ahCenterX + 6, HEADING_TAPE_HEIGHT + 6); ctx.closePath(); ctx.fill(); ctx.restore(); }
        function drawAirspeedTape(speed = 0) { const ctx = asiCtx; const tapeWidth = TAPE_WIDTH; const tapeHeight = TAPE_HEIGHT; const centerY = TAPE_CENTER_Y; const pixelsPerUnit = ASI_PIXELS_PER_UNIT; const pointerWidth = 15; ctx.save(); ctx.clearRect(0, 0, tapeWidth, tapeHeight); const pointerBoxHeight = 20; const pointerBoxY = centerY - pointerBoxHeight / 2; const textBoxX = 5; const textBoxWidth = tapeWidth - pointerWidth - 10; ctx.strokeStyle = PFD_TAPE_POINTER_COLOR; ctx.lineWidth = 1.5; ctx.strokeRect(textBoxX, pointerBoxY, textBoxWidth, pointerBoxHeight); ctx.fillStyle = PFD_TAPE_POINTER_COLOR; ctx.font = "bold 14px monospace"; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(speed.toFixed(0), textBoxX + textBoxWidth / 2, centerY); ctx.fillStyle = PFD_TAPE_POINTER_COLOR; ctx.beginPath(); ctx.moveTo(tapeWidth - pointerWidth, pointerBoxY); ctx.lineTo(tapeWidth, centerY); ctx.lineTo(tapeWidth - pointerWidth, pointerBoxY + pointerBoxHeight); ctx.closePath(); ctx.fill(); ctx.translate(0, centerY + speed * pixelsPerUnit); ctx.strokeStyle = PFD_TAPE_TICK_COLOR; ctx.fillStyle = PFD_TAPE_TICK_COLOR; ctx.font = "12px monospace"; ctx.textAlign = "right"; ctx.textBaseline = "middle"; const range = Math.ceil(tapeHeight / (2 * pixelsPerUnit * 5)) * 5; const startValue = Math.max(0, Math.floor(speed - range) - 5); const endValue = Math.ceil(speed + range) + 10; for (let val = startValue; val <= endValue; val += 1) { const yPos = -val * pixelsPerUnit; if (yPos < -centerY - 20 || yPos > tapeHeight - centerY + 20) continue; let tickLength = 0; let isLabelTick = false; ctx.lineWidth = 1; if (val % 10 === 0) { tickLength = 15; isLabelTick = true; ctx.lineWidth = 1.5; } else if (val % 5 === 0) { tickLength = 10; } else { tickLength = 5; } if (tickLength > 0) { ctx.beginPath(); ctx.moveTo(0, yPos); ctx.lineTo(tickLength, yPos); ctx.stroke(); if (isLabelTick && val >= 0) { ctx.fillText(val.toString(), -5, yPos); } } } ctx.restore(); }
        function drawAltitudeTape(alt = 0) { const ctx = altCtx; const tapeWidth = TAPE_ALT_WIDTH; const tapeHeight = TAPE_HEIGHT; const centerY = TAPE_CENTER_Y; const pixelsPerUnit = ALT_PIXELS_PER_UNIT; const pointerWidth = 15; ctx.save(); ctx.clearRect(0, 0, tapeWidth, tapeHeight); const pointerBoxHeight = 20; const pointerBoxY = centerY - pointerBoxHeight / 2; const textBoxX = pointerWidth + 5; const textBoxWidth = tapeWidth - pointerWidth - 10; ctx.strokeStyle = PFD_TAPE_POINTER_COLOR; ctx.lineWidth = 1.5; ctx.strokeRect(textBoxX, pointerBoxY, textBoxWidth, pointerBoxHeight); ctx.fillStyle = PFD_TAPE_POINTER_COLOR; ctx.font = "bold 14px monospace"; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(alt.toFixed(0), textBoxX + textBoxWidth / 2, centerY); ctx.fillStyle = PFD_TAPE_POINTER_COLOR; ctx.beginPath(); ctx.moveTo(pointerWidth, pointerBoxY); ctx.lineTo(0, centerY); ctx.lineTo(pointerWidth, pointerBoxY + pointerBoxHeight); ctx.closePath(); ctx.fill(); ctx.translate(0, centerY + alt * pixelsPerUnit); ctx.strokeStyle = PFD_TAPE_TICK_COLOR; ctx.fillStyle = PFD_TAPE_TICK_COLOR; ctx.font = "12px monospace"; ctx.textAlign = "left"; ctx.textBaseline = "middle"; const range = Math.ceil(tapeHeight / (2 * pixelsPerUnit * 10)) * 10; const startValue = Math.floor(alt - range) - 10; const endValue = Math.ceil(alt + range) + 20; for (let val = startValue; val <= endValue; val += 2) { const yPos = -val * pixelsPerUnit; if (yPos < -centerY - 20 || yPos > tapeHeight - centerY + 20) continue; let tickLength = 0; let isLabelTick = false; ctx.lineWidth = 1; if (val % 100 === 0) { tickLength = 20; isLabelTick = true; ctx.lineWidth = 2; } else if (val % 20 === 0) { tickLength = 15; isLabelTick = true; ctx.lineWidth = 1.5; } else if (val % 10 === 0) { tickLength = 10; } else if (val % 2 === 0) { tickLength = 5;} if (tickLength > 0) { ctx.beginPath(); ctx.moveTo(tapeWidth, yPos); ctx.lineTo(tapeWidth - tickLength, yPos); ctx.stroke(); if (isLabelTick) { ctx.fillText(val.toString(), tapeWidth - tickLength - 25, yPos); } } } ctx.restore(); }

        // --- Helper Functions (Identical to v5.12) ---
        function addMessage(text, type = "info") { const p = document.createElement("p"); const timestamp = new Date().toLocaleTimeString(); p.textContent = \`[\${timestamp}] \${text}\`; p.className = \`msg-\${type}\`; messagesDiv.appendChild(p); messagesDiv.scrollTop = messagesDiv.scrollHeight; while (messagesDiv.children.length > 150) { messagesDiv.removeChild(messagesDiv.children[1]); } }
        function setControlsDisabled(disabled) { Object.values(commandControls).forEach(control => { if (control) control.disabled = disabled; }); if (modeSelect) modeSelect.disabled = disabled; if (takeoffAltInput) takeoffAltInput.disabled = disabled; document.querySelectorAll('.goto-controls input').forEach(input => input.disabled = disabled); if(commandControls.LOAD_MISSION) commandControls.LOAD_MISSION.disabled = disabled; if(commandControls.FIT_MISSION) commandControls.FIT_MISSION.disabled = disabled; if (!disabled) { if (modeSelect) modeSelect.disabled = false; } }

        // --- SocketIO Event Handlers (Identical to v5.12) ---
        socket.on("connect", () => {
            console.log("Socket Connected"); addMessage("Connected to backend.", "info");
            document.getElementById("status-connection").textContent = "Backend Connected"; document.getElementById("status-connection").className = "connected";
            waypointLayerGroup.clearLayers(); geofenceLayerGroup.clearLayers();
            if (missionPolyline && map.hasLayer(missionPolyline)) map.removeLayer(missionPolyline); missionPolyline = null;
        });
        socket.on("disconnect", () => {
            console.log("Socket Disconnected"); addMessage("Disconnected from backend.", "error");
            document.getElementById("status-connection").textContent = "Disconnected"; document.getElementById("status-connection").className = "disconnected"; armingStatusOverlay.textContent = "DISARMED"; armingStatusOverlay.classList.remove("pfd-armed"); batteryValueSpan.textContent = "--.-"; modeValueSpan.textContent = "------"; currentValueSpan.textContent = "--.-"; gpsFixValueSpan.textContent = "---"; gpsSatsValueSpan.textContent = "0"; gpsHdopValueSpan.textContent = "--.-"; latValueSpan.textContent = "--.------"; lonValueSpan.textContent = "--.------"; document.getElementById('heartbeat-indicator')?.classList.remove('connected-state'); ekfStatusSpan.textContent = 'N/A'; ekfStatusSpan.className = 'ekf-status ekf-error';
            drawAttitudeIndicator(0, 0, 0); drawAirspeedTape(0); drawAltitudeTape(0); setControlsDisabled(true);
            if (droneMarker) { map.removeLayer(droneMarker); droneMarker = null; } if (homeMarker) { map.removeLayer(homeMarker); homeMarker = null; }
            waypointLayerGroup.clearLayers(); geofenceLayerGroup.clearLayers(); if (missionPolyline && map.hasLayer(missionPolyline)) map.removeLayer(missionPolyline); missionPolyline = null; firstLocationUpdate = true;
        });
        socket.on("connect_error", (err) => { console.error("Socket Connection Error:", err); addMessage(\`Backend connection error: \${err.message}\`, "error"); document.getElementById("status-connection").textContent = "Connect Error"; document.getElementById("status-connection").className = "disconnected"; document.getElementById('heartbeat-indicator')?.classList.remove('connected-state'); ekfStatusSpan.textContent = 'ERROR'; ekfStatusSpan.className = 'ekf-status ekf-error'; setControlsDisabled(true); });
        socket.on("status_message", (data) => { addMessage(data.text, data.type); });
        socket.on("command_ack_received", (data) => { let message = \`CMD \${data.command_name}: \${data.result_text}\`; let type = 'ack'; if (data.result === 0) type = 'ack'; else if (data.result === -1) { message = \`CMD \${data.command_name}: Timeout\`; type = 'ack-timeout'; } else type = 'ack-fail'; addMessage(message, type); });
        socket.on("mavlink_message", (msg) => { if (msg?.mavpackettype === 'HEARTBEAT') { const heartIcon = document.getElementById('heartbeat-indicator'); if (heartIcon && !heartIcon.classList.contains('beating')) { heartIcon.classList.add('beating'); heartIcon.addEventListener('animationend', () => { heartIcon.classList.remove('beating'); }, { once: true }); } } });
        socket.on("command_result", (data) => { if (!data.success) { const control = commandControls[data.command]; if (control) { control.disabled = false; if (data.command === 'SET_MODE' && modeSelect) modeSelect.disabled = false; } } });

        // --- Telemetry Update Handler (Identical map/PFD update logic to v5.12) ---
        socket.on("telemetry_update", (state) => {
            const isConnected = state.connected;
            document.getElementById("status-connection").textContent = isConnected ? "Drone Connected" : "Drone Disconnected"; document.getElementById("status-connection").className = isConnected ? "connected" : "disconnected";
            setControlsDisabled(!isConnected);
            document.getElementById('heartbeat-indicator')?.classList.toggle('connected-state', isConnected);
             const ekfReport = state.ekf_status_report || 'N/A'; if (ekfStatusSpan) { ekfStatusSpan.textContent = ekfReport; ekfStatusSpan.className = 'ekf-status ' + (ekfReport === 'EKF OK' ? 'ekf-ok' : (ekfReport.includes('INIT') || ekfReport.includes('N/A') || ekfReport.includes('ERROR') ? 'ekf-error' : 'ekf-warn')); }
            const mode = (state.mode || "UNKNOWN").toUpperCase(); const voltage = state.battery_voltage || 0; const current = state.battery_current ?? -1; const armed = state.armed || false; const lat = state.lat ?? 0; const lon = state.lon ?? 0; const fix = state.gps_fix_type ?? 0; const sats = state.satellites_visible ?? 0; const hdop = state.hdop ?? 99.9; batteryValueSpan.textContent = voltage.toFixed(2); modeValueSpan.textContent = mode; armingStatusOverlay.textContent = armed ? "ARMED" : "DISARMED"; armingStatusOverlay.classList.toggle("pfd-armed", armed); currentValueSpan.textContent = current >= 0 ? current.toFixed(1) : "--.-"; let fixStr = "---"; switch (fix) { case 0: case 1: fixStr = "NO"; break; case 2: fixStr = "2D"; break; case 3: fixStr = "3D"; break; case 4: fixStr = "DGPS"; break; case 5: fixStr = "RTKf"; break; case 6: fixStr = "RTKx"; break; default: fixStr = \`?\${fix}\`; } gpsFixValueSpan.textContent = fixStr; gpsSatsValueSpan.textContent = sats >= 0 ? sats : "?"; gpsHdopValueSpan.textContent = hdop < 99 ? hdop.toFixed(1) : "--.-"; latValueSpan.textContent = lat.toFixed(7); lonValueSpan.textContent = lon.toFixed(7);
            const airspeed = state.airspeed || 0; const altRel = state.alt_rel || 0; const heading = state.heading || 0; const pitch = (state.pitch || 0) * RAD_TO_DEG; const roll = (state.roll || 0) * RAD_TO_DEG; drawAttitudeIndicator(pitch, roll, heading); drawAirspeedTape(airspeed); drawAltitudeTape(altRel);
            if (isConnected && lat && lon && lat !== 0 && lon !== 0) {
                 const latLng = [lat, lon];
                 if (!droneMarker) droneMarker = L.marker(latLng, { icon: droneIcon }).addTo(map).bindPopup("Drone Position"); else droneMarker.setLatLng(latLng); // <-- THIS LINE UPDATES THE MARKER
                 if (firstLocationUpdate) { map.setView(latLng, 16); firstLocationUpdate = false; }
             }
            const homeLat = state.home_lat; const homeLon = state.home_lon;
             if (isConnected && homeLat && homeLon && homeLat !== 0 && homeLon !== 0) {
                 const homeLatLng = [homeLat, homeLon];
                 if (!homeMarker) homeMarker = L.marker(homeLatLng, { icon: homeIcon }).addTo(map).bindPopup("Home Position"); else homeMarker.setLatLng(homeLatLng);
             }
        });

        // --- Mission Update Handler (Identical to v5.12) ---
        socket.on("mission_update", (waypoints) => {
            console.log("Received mission update (Waypoints):", waypoints); waypointLayerGroup.clearLayers();
            if (missionPolyline && map.hasLayer(missionPolyline)) { map.removeLayer(missionPolyline); } if (missionPolyline && waypointLayerGroup.hasLayer(missionPolyline)) { waypointLayerGroup.removeLayer(missionPolyline); } missionPolyline = null;
            if (!waypoints || !Array.isArray(waypoints) || waypoints.length === 0) { addMessage("Mission list cleared or empty.", "info"); return; }
            const missionPoints = []; let validPointCount = 0;
            waypoints.forEach(wp => { if (wp.lat != null && wp.lon != null && !isNaN(wp.lat) && !isNaN(wp.lon)) { const latLng = L.latLng(wp.lat, wp.lon); missionPoints.push(latLng); validPointCount++; const wpIcon = L.divIcon({ className: 'waypoint-div-icon', html: wp.seq, iconSize: [18, 18], iconAnchor: [9, 9] }); const marker = L.marker(latLng, { icon: wpIcon }); let popupContent = \`<b>WP \${wp.seq}</b><br>Cmd: \${wp.command}<br>Lat: \${wp.lat.toFixed(6)}<br>Lon: \${wp.lon.toFixed(6)}<br>Alt: \${wp.alt.toFixed(1)}m (\${wp.frame})\`; marker.bindPopup(popupContent); marker.addTo(waypointLayerGroup); } else { console.warn("Skipping waypoint due to invalid coordinates:", wp); } });
            if (missionPoints.length > 1) { missionPolyline = L.polyline(missionPoints, { color: 'blue', weight: 2, opacity: 0.7 }); missionPolyline.addTo(waypointLayerGroup); }
            addMessage(\`Displayed \${validPointCount} waypoints.\`, "info");
        });

        // --- Geofence Update Handler (Identical to v5.12) ---
        socket.on("geofence_update", (fencePoints) => {
             console.log("Received geofence update:", fencePoints); geofenceLayerGroup.clearLayers();
            if (!fencePoints || !Array.isArray(fencePoints) || fencePoints.length === 0) { addMessage("Geofence list cleared or empty.", "info"); return; }
            const fenceCoords = []; let validPointCount = 0;
            fencePoints.forEach(fp => { if (fp.lat != null && fp.lon != null && !isNaN(fp.lat) && !isNaN(fp.lon)) { fenceCoords.push([fp.lat, fp.lon]); validPointCount++; } else { console.warn("Skipping fence point due to invalid coordinates:", fp); } });
            if (fenceCoords.length >= 3) { const isExclusion = fencePoints[0]?.command === 5103; const fenceColor = isExclusion ? '#ff8888' : '#88ff88'; const fenceOutline = isExclusion ? 'red' : 'green'; const polygon = L.polygon(fenceCoords, { color: fenceOutline, weight: 2, fillColor: fenceColor, fillOpacity: 0.3 }); polygon.bindPopup(\`Geofence (\${fenceCoords.length} points) - \${isExclusion ? 'Exclusion' : 'Inclusion'}\`); polygon.addTo(geofenceLayerGroup); addMessage(\`Displayed geofence polygon (\${fenceCoords.length} points).\`, "info"); } else if (validPointCount > 0) { addMessage(\`Received \${validPointCount} geofence points, but need at least 3 for a polygon.\`, "warning"); fenceCoords.forEach((coord, index) => { L.circleMarker(coord, { radius: 4, color: 'orange', fillOpacity: 0.5 }).bindPopup(\`Fence Point \${index+1}\`).addTo(geofenceLayerGroup); }); } else { addMessage("Geofence data received but contained no valid coordinate points.", "warning"); }
        });

        // --- Command Button Event Listeners (Identical actions to v5.12) ---
        commandControls.ARM.addEventListener("click", function() { if(confirm("ARM drone?")){this.disabled=!0;socket.emit("send_command",{command:"ARM"})} });
        commandControls.DISARM.addEventListener("click", function() { if(confirm("DISARM drone?")){this.disabled=!0;socket.emit("send_command",{command:"DISARM"})} });
        commandControls.TAKEOFF.addEventListener("click", function() { const altInput=takeoffAltInput; const alt=parseFloat(altInput.value);if(isNaN(alt)||alt<=0||alt>1e3)return addMessage("Invalid takeoff altitude.","warning"),altInput.focus();if(confirm(\`Takeoff to \${alt}m?\`)){this.disabled=!0;socket.emit("send_command",{command:"TAKEOFF",altitude:alt})} });
        commandControls.LAND.addEventListener("click", function() { if(confirm("LAND drone?")){this.disabled=!0;socket.emit("send_command",{command:"LAND"})} });
        commandControls.RTL.addEventListener("click", function() { if(confirm("Return-To-Launch (RTL)?")){this.disabled=!0;socket.emit("send_command",{command:"RTL"})} });
        commandControls.SET_MODE.addEventListener("click", function() { const mode=modeSelect.value;if(!mode)return addMessage("Please select a flight mode.","warning");if(confirm(\`Set mode to \${mode}?\`)){this.disabled=!0;modeSelect.disabled=!0;socket.emit("send_command",{command:"SET_MODE",mode_string:mode})} });
        commandControls.GOTO.addEventListener("click", function() { const latInput=document.getElementById("goto-lat"),lonInput=document.getElementById("goto-lon"),altInput=document.getElementById("goto-alt");const lat=parseFloat(latInput.value),lon=parseFloat(lonInput.value),alt=parseFloat(altInput.value);let valid=!0,errMsg="Invalid GoTo: ";if(isNaN(lat)||lat<-90||lat>90){valid=!1;errMsg+=" Lat.";latInput.focus()}else if(isNaN(lon)||lon<-180||lon>180){valid=!1;errMsg+=" Lon.";lonInput.focus()}else if(isNaN(alt)||alt<-100||alt>5e3){valid=!1;errMsg+=" Alt.";altInput.focus()}if(!valid)return addMessage(errMsg,"warning");if(confirm(\`Go to Lat:\${lat.toFixed(6)},Lon:\${lon.toFixed(6)},Alt:\${alt.toFixed(1)}m?\`)){this.disabled=!0;socket.emit("send_command",{command:"GOTO",lat:lat,lon:lon,alt:alt})} });
        commandControls["clear-goto"].addEventListener("click", () => { document.getElementById("goto-lat").value="";document.getElementById("goto-lon").value="";document.getElementById("goto-alt").value="10";addMessage("GoTo fields cleared.","info") });
        commandControls["center-map"].addEventListener("click", () => { if(droneMarker){map.setView(droneMarker.getLatLng(),map.getZoom()<14?16:map.getZoom());addMessage("Map centered on drone.","info")}else addMessage("Drone location unavailable.","warning") });
        commandControls.LOAD_MISSION.addEventListener("click", function() { if (confirm("Load Mission and Geofence from Flight Controller? This will clear current map items.")) { this.disabled = true; addMessage("Requesting mission/fence download...", "info"); socket.emit("request_mission_download", { mission_type: 0 }); } });

        // --- Fit Mission Bounds Functions (Identical to v5.12) ---
        function getMissionBounds() { const bounds = L.latLngBounds([]); waypointLayerGroup.eachLayer(layer => { if (layer instanceof L.Marker) { bounds.extend(layer.getLatLng()); } else if (layer instanceof L.Polyline || layer instanceof L.Polygon) { if (layer.getBounds && typeof layer.getBounds === 'function' && layer.getBounds().isValid()) { bounds.extend(layer.getBounds()); } else if (layer.getLatLngs && typeof layer.getLatLngs === 'function') { const latLngs = layer.getLatLngs(); if (latLngs && latLngs.length > 0) { bounds.extend(L.latLngBounds(latLngs)); } } } }); geofenceLayerGroup.eachLayer(layer => { if (layer instanceof L.Marker) { bounds.extend(layer.getLatLng()); } else if (layer instanceof L.Polygon && typeof layer.getBounds === 'function' && layer.getBounds().isValid()) { bounds.extend(layer.getBounds()); } }); if (homeMarker && homeMarker.getLatLng()) { bounds.extend(homeMarker.getLatLng()); } if (droneMarker && droneMarker.getLatLng()) { bounds.extend(droneMarker.getLatLng()); } return bounds; }
        function fitMissionBounds() { const bounds = getMissionBounds(); if (bounds.isValid()) { map.fitBounds(bounds, { padding: [30, 30] }); addMessage("Map zoomed to fit mission items.", "info"); } else { addMessage("No mission items, drone, or home location found to fit bounds.", "warning"); } }
        commandControls.FIT_MISSION.addEventListener("click", fitMissionBounds);

        // --- Map Click Listener (Identical to v5.12) ---
        map.on("click", (e) => { const latStr=e.latlng.lat.toFixed(7),lonStr=e.latlng.lng.toFixed(7);document.getElementById("goto-lat").value=latStr;document.getElementById("goto-lon").value=lonStr;addMessage(\`Map click: Lat \${latStr}, Lon \${lonStr}\`,"info") });

        // --- Initial UI Setup Function (Identical to v5.12) ---
        function initializeUI() {
            addMessage("Initializing UI...", "info"); drawAttitudeIndicator(); drawAirspeedTape(); drawAltitudeTape();
            armingStatusOverlay.textContent = "DISARMED"; armingStatusOverlay.classList.remove("pfd-armed"); batteryValueSpan.textContent = "--.-"; modeValueSpan.textContent = "------"; currentValueSpan.textContent = "--.-"; gpsFixValueSpan.textContent = "---"; gpsSatsValueSpan.textContent = "0"; gpsHdopValueSpan.textContent = "--.-"; latValueSpan.textContent = "--.------"; lonValueSpan.textContent = "--.------"; document.getElementById('heartbeat-indicator')?.classList.remove('connected-state'); if(ekfStatusSpan) { ekfStatusSpan.textContent = 'INIT'; ekfStatusSpan.className = 'ekf-status ekf-error'; }
            setControlsDisabled(true);
            waypointLayerGroup.clearLayers(); geofenceLayerGroup.clearLayers(); if (missionPolyline && map.hasLayer(missionPolyline)) map.removeLayer(missionPolyline); if (missionPolyline && waypointLayerGroup.hasLayer(missionPolyline)) waypointLayerGroup.removeLayer(missionPolyline); missionPolyline = null;
            setTimeout(() => { try { map.invalidateSize(); } catch(e) {console.error("Map invalidate size error:", e)} }, 100);
        }
        initializeUI();
    </script>
</body>
</html>
EOF
)
# Write the HTML content to the file
echo "$INDEX_HTML_CONTENT" > "${APP_DIR}/templates/index.html"
# Set ownership
sudo chown ${INSTALL_USER}:${INSTALL_GROUP} "${APP_DIR}/templates/index.html"

print_info "Creating ${APP_DIR}/templates/mavlink_dump.html (v${SCRIPT_VERSION})..."
# Use temporary variable approach for dump HTML
DUMP_HTML_CONTENT=$(cat <<EOF
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0"><title>MAVLink Message Dump (Pi v${SCRIPT_VERSION})</title><link rel="stylesheet" href="/static/css/style.css"/><style>body{font-family:monospace;background-color:#f4f4f4;margin:0;padding:0;display:flex;flex-direction:column;height:100vh}.header{background-color:#333;color:#fff;padding:10px;margin:0;display:flex;justify-content:space-between;align-items:center}.header h1{margin:0;font-size:1.2em}.nav-link{margin-left:20px;font-size:.9em;color:#adf;text-decoration:none}.dump-container{display:flex;flex-direction:column;flex-grow:1;padding:10px;overflow:hidden}.dump-controls{padding-bottom:5px;text-align:right}.dump-controls label{font-size:.8em;margin-right:5px}.dump-controls input{vertical-align:middle}.dump-controls button{padding:3px 8px;font-size:.8em}#other-dump-area{flex-grow:1;background-color:#fff;border:1px solid #ccc;padding:5px;overflow-y:scroll;font-size:.8em;white-space:pre}</style></head><body><div class="header"><h1>MAVLink Message Dump (Pi v${SCRIPT_VERSION})</h1><div class="header-right"><a href="/" class="nav-link">[Back to Main Control]</a></div></div><div class="dump-container"><h2>Messages (<span id="other-msg-count">0</span>)</h2><div class="dump-controls"><label><input type="checkbox" id="filter-hb" checked> Filter HEARTBEAT</label> <button id="clear-button">Clear Log</button></div><pre id="other-dump-area">Waiting for messages... (NOTE: By default, only HEARTBEAT messages are broadcast for the UI pulse. Other messages require backend changes to enable dumping.)</pre></div><script src="/static/lib/socket.io.min.js"></script><script>// JS identical to v5.12
const otherDumpArea=document.getElementById("other-dump-area"),otherCountSpan=document.getElementById("other-msg-count"),clearButton=document.getElementById("clear-button"),filterHbCheckbox=document.getElementById("filter-hb");let otherMessageCount=0;const MAX_MESSAGES=500,socket=io.connect(location.protocol+"//"+document.domain+":"+location.port+"/drone");function clearArea(e,t){e.textContent="Log cleared. Waiting...\n(NOTE: By default, only HEARTBEAT messages are broadcast for the UI pulse.)\n",otherMessageCount=0,t.textContent=0}function appendToArea(e,t,o){const n=e.scrollHeight-e.clientHeight<=e.scrollTop+10;let l;try{l="object"==typeof t&&null!==t?JSON.stringify(t,null,2):String(t)}catch(e){l=String(t)}const c=document.createTextNode(l+"\n"),d=document.createTextNode("---\n");for(e.appendChild(c),e.appendChild(d),otherMessageCount++;e.childNodes.length>2*MAX_MESSAGES+1;)e.removeChild(e.firstChild),e.firstChild&&e.removeChild(e.firstChild),otherMessageCount--;o.textContent=otherMessageCount,n&&(e.scrollTop=e.scrollHeight)}socket.on("connect",()=>{console.log("Connected for dump"),otherDumpArea.textContent="Connected. Waiting...\n(NOTE: By default, only HEARTBEAT messages are broadcast for the UI pulse.)\n"}),socket.on("disconnect",()=>{console.log("Disconnected"),otherDumpArea.textContent+="\n*** Disconnected from backend. ***\n"}),socket.on("mavlink_message",e=>{if(e){let t="object"==typeof e&&null!==e?e.mavpackettype:void 0;filterHbCheckbox.checked&&"HEARTBEAT"===t||appendToArea(otherDumpArea,e,otherCountSpan)}}),clearButton.addEventListener("click",()=>clearArea(otherDumpArea,otherCountSpan));
</script></body></html>
EOF
)
echo "$DUMP_HTML_CONTENT" > "${APP_DIR}/templates/mavlink_dump.html"
sudo chown ${INSTALL_USER}:${INSTALL_GROUP} "${APP_DIR}/templates/mavlink_dump.html"


print_info "Creating ${APP_DIR}/static/css/style.css..."
# CSS is unchanged from v5.12, just copy it over.
# No variable expansion needed -> Use quoted 'EOF'
cat > "${APP_DIR}/static/css/style.css" << 'EOF'
/* CSS from v5.11 + waypoint marker style */
html{box-sizing:border-box}*,:after,:before{box-sizing:inherit}body{margin:0;padding:0;font-family:Arial,Helvetica,sans-serif;background-color:#f4f4f4;color:#333;font-size:14px;line-height:1.4}h1.main-title{flex-shrink:0;background-color:#333;color:#fff;padding:8px 15px;margin:0;font-size:1.2em}.main-container{display:flex;flex:1;overflow:hidden}.left-column{display:flex;flex-direction:column;flex:0 0 480px;padding:10px;border-right:1px solid #ccc;overflow-y:auto;background-color:#f0f0f0}.right-column#map{flex:1;background-color:#e0e0e0;min-height:200px;position:relative}.pfd-section{background-color:#000;border:1px solid #555;border-radius:4px;margin-bottom:15px;flex-shrink:0;height:250px;overflow:hidden;position:relative}#attitude-indicator-wrapper{position:relative;width:100%;height:100%;overflow:hidden}#attitude-canvas{display:block;position:absolute;left:0;top:0;width:100%;height:100%;z-index:1}#alt-tape-container,#asi-tape-container{position:absolute;top:0;height:100%;z-index:5;pointer-events:none}#asi-tape-container{left:0;width:60px}#alt-tape-container{right:0;width:70px}#alt-canvas,#asi-canvas{display:block;width:100%;height:100%}.pfd-overlay{position:absolute;color:#fff;padding:2px 6px;font-size:10px;font-family:monospace;z-index:10;line-height:1.1;pointer-events:none;text-shadow:1px 1px 2px rgba(0,0,0,.7)}#arming-status-overlay{top:25%;left:50%;transform:translate(-50%,-50%);font-size:1.5em;font-weight:700;color:#28a745;text-shadow:1px 1px 2px rgba(0,0,0,.8);z-index:11;text-align:center;width:100%;padding:0}#arming-status-overlay.pfd-armed{color:#dc3545}#battery-overlay{bottom:5px;left:5px;font-size:11px}#mode-overlay{bottom:5px;right:5px;font-size:11px;font-weight:700;text-transform:uppercase;color:#0f0}#current-overlay{bottom:22px;left:5px;font-size:11px}#gps-overlay{top:30px;left:5px;color:#ffc107;font-size:10px}#latlon-overlay{top:30px;right:5px;text-align:right;font-size:10px}.info-controls-section{display:flex;flex-direction:column;flex:1;min-height:0}.status{margin-bottom:15px;flex-shrink:0}.status h2{font-size:1.1em;margin:0 0 8px;color:#333;border-bottom:1px solid #ddd;padding-bottom:4px}.status-grid{display:grid;grid-template-columns:auto;gap:6px}.status-item{background-color:#e9ecef;padding:5px 8px;border-radius:3px;font-size:.85em;overflow-wrap:break-word;display:flex;align-items:center}.status-item strong{display:inline-block;min-width:90px;font-size:1em;color:#495057;margin-bottom:0;text-transform:uppercase;margin-right:5px;text-align:right}#messages{height:90px;max-height:150px;overflow-y:auto;border:1px solid #ced4da;padding:5px 8px;margin-top:8px;background-color:#fff;font-size:.8em;border-radius:3px;line-height:1.2}#messages p{margin:1px 0;padding:1px 0;border-bottom:1px dashed #eee}#messages p:last-child{border-bottom:none}.msg-ack,.msg-info{color:#17a2b8}.msg-error,.msg-warning{font-weight:700}.msg-warning{color:#ffc107}.msg-error{color:#dc3545}.msg-ack{color:#007bff}.msg-ack-fail{color:#dc3545;font-weight:700}.msg-ack-timeout{color:#fd7e14;font-weight:700}.controls{background-color:#ddd;padding:10px;border-radius:5px;border:1px solid #bbb;flex-shrink:0}.controls h2{font-size:1.1em;margin:0 0 8px;color:#333;border-bottom:1px solid #bbb;padding-bottom:4px}.controls div{margin-bottom:8px}.controls button,.controls select{margin:2px 3px 2px 0;padding:6px 10px;font-size:.9em;cursor:pointer;border:1px solid #999;border-radius:3px;background-color:#007bff;color:#fff;transition:background-color .2s ease,color .2s ease,opacity .2s ease;vertical-align:middle}.controls select{background-color:#fff;color:#333;padding:5px;height:31px}.controls button:active:not(:disabled){background-color:#004085}.controls button:hover:not(:disabled){background-color:#0056b3}.controls button:disabled,.controls select:disabled{background-color:#ccc;color:#666;cursor:not-allowed;opacity:.7;border-color:#bbb}.controls input[type=number],.controls input[type=text]{padding:5px 7px;margin:0 4px 4px 0;border:1px solid #aaa;border-radius:3px;width:75px;font-size:.9em;height:31px;vertical-align:middle}.controls input[type=number]{width:55px}.controls hr{margin:10px 0;border:0;border-top:1px solid #aaa}.mode-select-control label{margin-right:5px;font-weight:700}.mode-select-control{margin-left:10px}.goto-controls{display:flex;align-items:center;flex-wrap:wrap;gap:4px}.goto-controls input{flex-grow:1;min-width:60px}#btn-clear-goto{background-color:#6c757d}#btn-clear-goto:hover:not(:disabled){background-color:#5a6268}.connected{color:#28a745;font-weight:700}.disconnected{color:#dc3545;font-weight:700}.heartbeat-icon{display:inline-block;margin-left:5px;vertical-align:middle;font-size:13px;color:#dc3545;transition:color .5s ease;transform-origin:center center;-webkit-user-select:none;-ms-user-select:none;user-select:none}.heartbeat-icon.connected-state{color:#2ecc71}@keyframes beat{0%{transform:scale(1)}50%{transform:scale(1.5)}to{transform:scale(1)}}.heartbeat-icon.beating{animation:beat .35s ease-in-out}.ekf-status{font-weight:700;margin-left:5px}.ekf-ok{color:#28a745}.ekf-warn{color:#ffc107}.ekf-error{color:#dc3545}#dump-link-button{position:fixed;bottom:8px;right:8px;padding:5px 8px;background-color:rgba(50,50,50,.7);color:#fff;text-decoration:none;border-radius:3px;font-size:.75em;border:1px solid #222;z-index:1000}#dump-link-button:hover{background-color:rgba(0,0,0,.8)}.map-controls{position:absolute;top:10px;left:50px;z-index:500;background:rgba(255,255,255,.8);padding:5px;border-radius:4px;border:1px solid #bbb}.map-controls button{font-size:11px;padding:3px 6px;margin-left:5px;background-color:#fff;color:#333;border:1px solid #aaa}.map-controls button:hover:not(:disabled){background-color:#eee}
/* Style added in v5.12 for waypoint markers */
.waypoint-div-icon {
    background: #007bff; /* Blue background */
    color: white;       /* White text */
    border-radius: 50%; /* Circular shape */
    text-align: center;
    line-height: 18px;  /* Center text vertically */
    font-size: 10px;
    font-weight: bold;
    border: 1px solid white; /* White border */
    box-shadow: 1px 1px 3px rgba(0,0,0,0.5); /* Slight shadow */
    width: 18px;        /* Fixed size */
    height: 18px;       /* Fixed size */
}
/* Mission controls section style added in v5.12 */
.mission-controls {
    margin-top: 10px;
    padding-top: 10px;
    border-top: 1px solid #bbb;
}
.mission-controls button {
     background-color: #ffc107; /* Orange color for mission button */
     color: #333;
     margin-right: 10px;
}
 .mission-controls button:hover:not(:disabled) {
     background-color: #e0a800;
}

@media (max-width:850px){.main-container{flex-direction:column}.left-column{flex-basis:auto;border-right:none;border-bottom:1px solid #ccc;max-height:65vh;width:100%;flex:0 0 auto;padding:5px}.right-column#map{flex:1;min-height:250px}.pfd-section{height:200px;margin-bottom:10px}#attitude-canvas{width:100%;height:100%}#alt-canvas,#asi-canvas{height:100%}}@media (max-width:520px){.left-column{max-height:70vh;flex-basis:auto}.pfd-section{height:180px}.status-grid{grid-template-columns:auto}.controls button,.controls select{font-size:.85em;padding:5px 8px}.controls input{width:60px;height:29px}.controls input[type=number]{width:45px}.pfd-overlay{font-size:9px;padding:1px 4px}#arming-status-overlay{font-size:1.3em;top:25%}.goto-controls input{min-width:50px}}
EOF
# Set ownership for the created file
sudo chown ${INSTALL_USER}:${INSTALL_GROUP} "${APP_DIR}/static/css/style.css"


# --- 7. Create Drone Control Web Application Service ---
print_info "Creating systemd service for the Drone Control Web App (v${SCRIPT_VERSION})..."
# Needs variable expansion for user, paths, version -> Use unquoted EOF
sudo bash -c "cat > /etc/systemd/system/drone_control_web.service" << EOF
[Unit]
Description=Drone Control Web Application Service (v${SCRIPT_VERSION})
# Start after networking is up and mavlink-router is running (or supposed to be)
After=network.target mavlink-router.service
Wants=mavlink-router.service

[Service]
# Run as the specified user and group
User=${INSTALL_USER}
Group=${INSTALL_GROUP}
# Set the working directory for the application
WorkingDirectory=${APP_DIR}
# Command to execute: use the python from the virtual environment
ExecStart=${VENV_PATH}/bin/python ${APP_DIR}/app.py
# Restart the service automatically if it crashes
Restart=always
RestartSec=10 # Wait 10 seconds before restarting
# Log output to systemd journal
StandardOutput=journal
StandardError=journal

[Install]
# Enable the service to start on boot in multi-user mode
WantedBy=multi-user.target
EOF


# --- 8. Add User to dialout Group ---
print_info "Adding user ${INSTALL_USER} to 'dialout' group for serial port access..."
# Check if user is already in the group
if groups ${INSTALL_USER} | grep -q '\bdialout\b'; then
    print_info "User ${INSTALL_USER} already in dialout group."
else
    sudo usermod -a -G dialout ${INSTALL_USER}
    print_info "Added user ${INSTALL_USER} to dialout group."
    print_warning "Group change for ${INSTALL_USER} will take effect after next login or reboot."
fi


# --- 9. Enable New Service & Reload Daemon ---
print_info "Reloading systemd daemon and enabling drone_control_web service..."
sudo systemctl daemon-reload
sudo systemctl enable drone_control_web.service
# Note: We don't start drone_control_web.service here because mavlink-router
# and serial port changes require a reboot to function correctly.
# Starting it now would likely cause it to fail or not connect properly.


# --- Final Instructions ---
echo ""
echo "============================================================="
echo " Installation Script Finished (v${SCRIPT_VERSION} - Map Update Fix) "
echo "============================================================="
echo ""
print_warning "--> MANDATORY STEP: A REBOOT IS REQUIRED! <--"
echo ""
echo "   A reboot is necessary for:"
echo "     - Serial port configuration changes (/boot/*.txt)."
echo "     - User ('${INSTALL_USER}') group membership ('dialout')."
echo "     - Systemd services to start reliably in the correct order."
echo ""
echo "   Run: sudo reboot"
echo ""
echo "============================================================="
echo " AFTER REBOOTING: "
echo "============================================================="
echo ""
echo "1. VERIFY SERVICES:"
echo "   - Connect your Flight Controller to ${SERIAL_PORT} and power it on."
echo "   - Check Mavlink Router: sudo systemctl status mavlink-router.service"
echo "   - Check Web App:      sudo systemctl status drone_control_web.service"
echo "   - Both should show 'active (running)'."
echo "   - If issues:"
echo "     - Router Logs:   journalctl -u mavlink-router -n 50 -f"
echo "     - Web App Logs:  journalctl -u drone_control_web -n 50 -f"
echo "     - Ensure FC is properly connected and powered."
echo "     - **CRITICAL: Verify baud rate in /etc/mavlink-router/main.conf ('Baud = ${FC_BAUD_RATE}') matches your Flight Controller's serial port setting!**"
echo ""
echo "2. ACCESS WEB UI (v${SCRIPT_VERSION}):"
echo "   - The application should now be running automatically."
echo "   - On the Pi's Browser: http://127.0.0.1:5000"
echo "   - From another computer on the same network: http://<raspberry_pi_ip>:5000"
echo "     (Find Pi's IP address using: ip addr show wlan0 | grep 'inet ')"
echo ""
echo "3. VERIFY FIX:"
echo "   - After connecting, check if the drone marker on the map updates its position"
echo "     as the drone moves (or as simulated data is received)."
echo ""
echo "4. MANAGING SERVICES:"
echo "   - Stop Router:       sudo systemctl stop mavlink-router.service"
echo "   - Stop Web App:      sudo systemctl stop drone_control_web.service"
echo "   - Start Router:      sudo systemctl start mavlink-router.service"
echo "   - Start Web App:     sudo systemctl start drone_control_web.service"
echo "   - Restart Web App:   sudo systemctl restart drone_control_web.service"
echo "   - Disable Auto-start: sudo systemctl disable drone_control_web.service"
echo "   - Re-enable Auto-start: sudo systemctl enable drone_control_web.service"
echo ""
echo "============================================================="

exit 0
