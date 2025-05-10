#!/bin/bash

# Record start time
START_TIME=$(date +%s)
START_TIME_HUMAN=$(date)
echo "Installation started at: $START_TIME_HUMAN"

# ==============================================================================
# Drone Control System Installation Script
# Target Platform: Raspberry Pi OS (Raspbian)
# Components:
# - MAVLink Router
# - WebGCS
# - Python Environment Setup
# - Systemd Service Configuration
# ==============================================================================

set -e # Exit immediately if a command exits with a non-zero status

# --- Configuration ---
PYTHON_VERSION="python3"
PIP_VERSION="pip3"
WEBGCS_DIR="/home/pi/WebGCS"
MAVLINK_ROUTER_DIR="/home/pi/installmavlinkrouter2024"
SERIAL_PORT="/dev/serial0"
FC_BAUD_RATE="57600"
MAVLINK_ROUTER_UDP_PORT="14550"

# --- Cleanup Function ---
cleanup_old_installation() {
    print_info "Cleaning up old installation..."
    
    # Stop services if they exist
    systemctl stop mavlink-router webgcs create_ap 2>/dev/null || true
    systemctl disable mavlink-router webgcs create_ap 2>/dev/null || true
    
    # Remove old service files
    rm -f /etc/systemd/system/mavlink-router.service
    rm -f /etc/systemd/system/webgcs.service
    rm -f /etc/systemd/system/create_ap.service
    
    # Remove old config files
    rm -f /etc/mavlink-router/main.conf
    
    # Remove old installations
    rm -rf "$WEBGCS_DIR"
    rm -rf "/home/pi/RaspberryPiHotspotIfNoWifi"
    
    # Reload systemd to recognize removed services
    systemctl daemon-reload
    
    print_info "Cleanup completed"
}

# --- Helper Functions ---
print_info() { echo "[INFO] $1"; }
print_error() { echo "[ERROR] $1" >&2; }

# Function to check internet connectivity
check_internet() {
    print_info "Checking internet connectivity..."
    for i in {1..30}; do
        if ping -c 1 8.8.8.8 >/dev/null 2>&1; then
            print_info "Internet connection established"
            return 0
        fi
        print_info "Waiting for internet connection... (attempt $i/30)"
        sleep 2
    done
    print_error "No internet connection after 30 attempts"
    return 1
}

# Function to safely clone a git repository with retries
safe_git_clone() {
    local repo_url="$1"
    local target_dir="$2"
    local max_attempts=3
    
    for attempt in $(seq 1 $max_attempts); do
        print_info "Cloning $repo_url (attempt $attempt/$max_attempts)"
        if git clone "$repo_url" "$target_dir"; then
            return 0
        fi
        print_error "Git clone failed, checking internet connection..."
        check_internet || return 1
        sleep 5
    done
    print_error "Failed to clone repository after $max_attempts attempts"
    return 1
}

# --- Check for sudo ---
if [ "$EUID" -ne 0 ]; then
    print_error "Please run as root (use sudo)"
    exit 1
fi

# --- 1. Clean Up Old Installation ---
cleanup_old_installation

# --- 2. System Updates and Dependencies ---
print_info "Updating system packages..."
apt-get update
apt-get upgrade -y
apt-get install -y git python3 python3-pip python3-venv build-essential meson ninja-build pkg-config curl

# Enable immediate history writing
echo "export PROMPT_COMMAND='history -a'" | sudo tee -a /etc/bash.bashrc

# --- 3. Configure UART for Flight Controller ---
print_info "Configuring serial port..."

# Set configuration paths for Ubuntu on Raspberry Pi
CONFIG_FILE="/boot/firmware/config.txt"
CMDLINE_FILE="/boot/firmware/cmdline.txt"

# Configure UART in config.txt
if [ -f "$CONFIG_FILE" ]; then
    cp "$CONFIG_FILE" "${CONFIG_FILE}.bak"
    
    # Remove existing uart settings
    sed -i '/^enable_uart=/d' "$CONFIG_FILE"
    sed -i '/^dtoverlay=uart/d' "$CONFIG_FILE"
    sed -i '/^dtoverlay=pi3-disable-bt/d' "$CONFIG_FILE"
    sed -i '/^dtoverlay=disable-bt/d' "$CONFIG_FILE"
    sed -i '/^dtparam=uart0=/d' "$CONFIG_FILE"
    sed -i '/^dtparam=uart1=/d' "$CONFIG_FILE"
    
    # Add UART configuration
    echo "" >> "$CONFIG_FILE"
    echo "# UART Configuration" >> "$CONFIG_FILE"
    echo "enable_uart=1" >> "$CONFIG_FILE"
    echo "dtparam=uart0=on" >> "$CONFIG_FILE"
    echo "dtparam=uart1=off" >> "$CONFIG_FILE"
    echo "dtoverlay=disable-bt" >> "$CONFIG_FILE"
    print_info "Updated config.txt with UART settings (backup saved)"
fi

# Remove serial console from cmdline.txt
if [ -f "$CMDLINE_FILE" ]; then
    cp "$CMDLINE_FILE" "${CMDLINE_FILE}.bak"
    sed -i 's/console=ttyAMA0,[0-9]\+ //g' "$CMDLINE_FILE"
    sed -i 's/console=serial0,[0-9]\+ //g' "$CMDLINE_FILE"
    print_info "Updated cmdline.txt (backup saved)"
fi

# Add user to dialout group
if ! groups $SUDO_USER | grep -q dialout; then
    usermod -a -G dialout $SUDO_USER
    print_info "Added $SUDO_USER to dialout group (will take effect after next login)"
fi

print_info "Serial port configuration complete. Reboot required for changes to take effect."

# --- 4. Install Raspberry Pi Hotspot Failover ---
print_info "Installing Raspberry Pi Hotspot Failover..."

# Install required packages
print_info "Installing required packages..."
apt-get install -y hostapd dnsmasq

# Clone repository
cd /home/pi
if [ -d "RaspberryPiHotspotIfNoWifi" ]; then
    rm -rf "RaspberryPiHotspotIfNoWifi"
fi
if ! safe_git_clone "https://github.com/PeterJBurke/RaspberryPiHotspotIfNoWifi.git" "RaspberryPiHotspotIfNoWifi"; then
    print_error "Failed to install WiFi Hotspot Failover. Please check your internet connection and try again."
    exit 1
fi
cd RaspberryPiHotspotIfNoWifi

# Install the check_wifi service and script
print_info "Installing WiFi check service..."
cp check_wifi.service /etc/systemd/system/
cp check_wifi.sh /usr/local/bin/
chmod +x /usr/local/bin/check_wifi.sh

# Enable and start the service
systemctl daemon-reload
systemctl enable check_wifi.service
systemctl start check_wifi.service

print_info "WiFi Hotspot Failover installation completed"

# --- 5. Install MAVLink Router ---
print_info "Installing MAVLink Router..."
cd /home/pi
if [ -d "$MAVLINK_ROUTER_DIR" ]; then
    rm -rf "$MAVLINK_ROUTER_DIR"
fi
if ! safe_git_clone "https://github.com/PeterJBurke/installmavlinkrouter2024.git" "installmavlinkrouter2024"; then
    print_error "Failed to install MAVLink Router. Please check your internet connection and try again."
    exit 1
fi
cd installmavlinkrouter2024
chmod +x install.sh
./install.sh

# --- 6. Install WebGCS ---
print_info "Installing WebGCS..."
cd /home/pi
if [ -d "$WEBGCS_DIR" ]; then
    rm -rf "$WEBGCS_DIR"
fi
if ! safe_git_clone "https://github.com/PeterJBurke/WebGCS.git" "WebGCS"; then
    print_error "Failed to install WebGCS. Please check your internet connection and try again."
    exit 1
fi
cd WebGCS

# Create WebGCS config
print_info "Creating WebGCS configuration..."
cat > "${WEBGCS_DIR}/config.py" << EOF
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Drone Connection Settings
DRONE_TCP_ADDRESS = os.getenv('DRONE_TCP_ADDRESS', '127.0.0.1')
DRONE_TCP_PORT = os.getenv('DRONE_TCP_PORT', '5678')
MAVLINK_CONNECTION_STRING = f'tcp:{DRONE_TCP_ADDRESS}:{DRONE_TCP_PORT}'

# Web Server Settings
WEB_SERVER_HOST = os.getenv('WEB_SERVER_HOST', '0.0.0.0')  # Listen on all interfaces
WEB_SERVER_PORT = int(os.getenv('WEB_SERVER_PORT', '5000'))
SECRET_KEY = os.getenv('SECRET_KEY', 'desktop_drone_secret!')

# MAVLink Settings
HEARTBEAT_TIMEOUT = int(os.getenv('HEARTBEAT_TIMEOUT', '30'))
REQUEST_STREAM_RATE_HZ = int(os.getenv('REQUEST_STREAM_RATE_HZ', '4'))
COMMAND_ACK_TIMEOUT = int(os.getenv('COMMAND_ACK_TIMEOUT', '10'))
TELEMETRY_UPDATE_INTERVAL = float(os.getenv('TELEMETRY_UPDATE_INTERVAL', '0.1'))
EOF

# --- 7. Create Python Virtual Environment ---
print_info "Setting up Python virtual environment..."
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
deactivate

# --- 8. Configure MAVLink Router Service ---
print_info "Configuring MAVLink Router service..."

# Ensure config directory exists
mkdir -p /etc/mavlink-router

# Create common serial device dependency file for WebGCS
print_info "Creating common serial device dependency configuration..."
cat > /etc/systemd/system/serial-device.conf << EOF
[Unit]
ConditionPathExists=/dev/serial0
After=dev-serial0.device sys-devices-platform-serial0-tty-ttyAMA0.device
Requires=dev-serial0.device sys-devices-platform-serial0-tty-ttyAMA0.device
EOF

# Create MAVLink Router config
# Download the proven working configuration from installmavlinkrouter2024
print_info "Downloading MAVLink Router configuration..."
wget https://raw.githubusercontent.com/PeterJBurke/installmavlinkrouter2024/main/main.conf -O /etc/mavlink-router/main.conf
chmod 644 /etc/mavlink-router/main.conf

# --- 9. Create WebGCS Service ---
print_info "Creating WebGCS service..."

# Create WebGCS service
cat > /lib/systemd/system/webgcs.service << EOF
[Unit]
Description=WebGCS Service
# Include common serial device dependencies
.include /etc/systemd/system/serial-device.conf
After=network.target mavlink-router.service
Requires=mavlink-router.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/WebGCS
Environment=PATH=/home/pi/WebGCS/venv/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ExecStart=/home/pi/WebGCS/venv/bin/python3 app.py
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target
EOF

# --- 7. Enable and Start Services ---
print_info "Enabling and starting services..."
systemctl daemon-reload
systemctl enable mavlink-router.service
systemctl enable webgcs.service
systemctl start mavlink-router.service
systemctl start webgcs.service

# --- 6. Configure UART ---
print_info "Configuring UART..."

# Initialize UART configuration flag
UART_CONFIGURED=0

# Configure UART in config.txt
if ! grep -q "enable_uart=1" /boot/config.txt; then
    echo "enable_uart=1" >> /boot/config.txt
    UART_CONFIGURED=1
fi
if ! grep -q "dtoverlay=disable-bt" /boot/config.txt; then
    echo "dtoverlay=disable-bt" >> /boot/config.txt
    UART_CONFIGURED=1
fi

# --- 9. Set Permissions ---
print_info "Setting permissions..."
chown -R pi:pi "$WEBGCS_DIR"
usermod -a -G dialout pi

# Calculate and display duration
END_TIME=$(date +%s)
END_TIME_HUMAN=$(date)
DURATION=$((END_TIME - START_TIME))
HOURS=$((DURATION / 3600))
MINUTES=$(( (DURATION % 3600) / 60 ))
SECONDS=$((DURATION % 60))

echo -e "\nInstallation timing summary:"
echo "Started : $START_TIME_HUMAN"
echo "Finished: $END_TIME_HUMAN"
echo "Duration: ${HOURS}h ${MINUTES}m ${SECONDS}s"

print_info "Installation completed successfully!"

if [ "$UART_CONFIGURED" -eq 1 ]; then
    print_info "IMPORTANT: You MUST reboot for the UART configuration changes to take effect."
    print_info "After reboot, the serial devices (/dev/serial0 and /dev/ttyAMA0) will be available."
    print_info "The MAVLink Router and WebGCS services will start automatically after reboot."
    
    # Disable services until after reboot
    systemctl disable mavlink-router
    systemctl disable webgcs
    
    print_info "Services will be enabled at next boot"
    
    # Create systemd oneshot service to enable services after reboot
    cat > /etc/systemd/system/enable-drone-services.service << EOF
[Unit]
Description=Enable Drone Control Services After Reboot
# Include common serial device dependencies
.include /etc/systemd/system/serial-device.conf

[Service]
Type=oneshot
ExecStart=/bin/systemctl enable mavlink-router
ExecStart=/bin/systemctl start mavlink-router
ExecStart=/bin/systemctl enable webgcs
ExecStart=/bin/systemctl start webgcs
ExecStart=/bin/systemctl disable enable-drone-services
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
    
    # Enable the oneshot service
    systemctl enable enable-drone-services
else
    print_info "A reboot is recommended but not required."
fi

print_info "After reboot:"
print_info "1. MAVLink Router will run automatically"
print_info "2. WebGCS will be available at http://[raspberry-pi-ip]:5000"
print_info "3. UART will be configured for the flight controller"
echo
print_info "To reboot now, type: sudo reboot"
