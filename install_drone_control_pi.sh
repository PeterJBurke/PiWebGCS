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
    rm -f /etc/create_ap.conf
    
    # Remove old installations
    rm -rf "$WEBGCS_DIR"
    rm -rf "$MAVLINK_ROUTER_DIR"
    rm -rf "/home/pi/RaspberryPiHotspotIfNoWifi"
    
    # Reload systemd to recognize removed services
    systemctl daemon-reload
    
    print_info "Cleanup completed"
}

# --- Helper Functions ---
print_info() { echo "[INFO] $1"; }
print_error() { echo "[ERROR] $1" >&2; }

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
print_info "Configuring UART interface..."

# Check if UART is already enabled
UART_CONFIGURED=0
if grep -q "^enable_uart=1" /boot/config.txt && grep -q "^dtoverlay=disable-bt" /boot/config.txt; then
    print_info "UART already configured in /boot/config.txt"
else
    print_info "Configuring UART in /boot/config.txt"
    # Remove any existing uart/bluetooth config lines to avoid duplicates
    sed -i '/^enable_uart=/d' /boot/config.txt
    sed -i '/^dtoverlay=disable-bt/d' /boot/config.txt
    
    # Add our configuration
    echo "dtoverlay=disable-bt" | sudo tee -a /boot/config.txt
    echo "enable_uart=1" | sudo tee -a /boot/config.txt
    UART_CONFIGURED=1
fi

# Stop and disable Bluetooth services
systemctl stop bluetooth 2>/dev/null || true
systemctl disable bluetooth 2>/dev/null || true

# Try to set up serial devices, but don't fail if they don't exist yet
print_info "Setting up serial devices (will be available after reboot if not present)"

# Create serial device symbolic link if possible
if [ -e "/dev/ttyAMA0" ]; then
    ln -sf /dev/ttyAMA0 /dev/serial0 2>/dev/null || true
    chmod 666 /dev/ttyAMA0 2>/dev/null || true
fi

if [ -e "/dev/serial0" ]; then
    chmod 666 /dev/serial0 2>/dev/null || true
fi

# If we made UART config changes, we need a reboot
if [ "$UART_CONFIGURED" -eq 1 ]; then
    print_info "UART configuration has been updated. A reboot will be required after installation."
    REBOOT_REQUIRED=1
fi

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
git clone https://github.com/PeterJBurke/RaspberryPiHotspotIfNoWifi.git
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
git clone https://github.com/PeterJBurke/installmavlinkrouter2024.git
cd installmavlinkrouter2024
chmod +x install.sh
./install.sh

# --- 6. Install WebGCS ---
print_info "Installing WebGCS..."
cd /home/pi
if [ -d "$WEBGCS_DIR" ]; then
    rm -rf "$WEBGCS_DIR"
fi
git clone https://github.com/PeterJBurke/WebGCS.git
cd WebGCS

# --- 7. Create Python Virtual Environment ---
print_info "Setting up Python virtual environment..."
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
deactivate

# --- 8. Configure MAVLink Router Service ---
print_info "Configuring MAVLink Router service..."
cat > /etc/mavlink-router/main.conf << EOF
[General]
MavlinkSysid = 254
UdpEndpoints = gcs_app
UartEndpoints = fc_serial

[UartEndpoint fc_serial]
Device = ${SERIAL_PORT}
Baud = ${FC_BAUD_RATE}
FlowControl = false

[UdpEndpoint gcs_app]
Mode = Normal
Address = 127.0.0.1
Port = ${MAVLINK_ROUTER_UDP_PORT}
EOF

# --- 9. Create WebGCS Service ---
print_info "Creating WebGCS service..."
cat > /etc/systemd/system/webgcs.service << EOF
[Unit]
Description=WebGCS Service
After=network.target mavlink-router.service
Requires=mavlink-router.service

[Service]
Type=simple
User=pi
WorkingDirectory=${WEBGCS_DIR}
Environment=PATH=${WEBGCS_DIR}/venv/bin:$PATH
ExecStart=${WEBGCS_DIR}/venv/bin/python app.py
Restart=always
RestartSec=5

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

# --- 8. Configure UART ---
print_info "Configuring UART..."
if ! grep -q "enable_uart=1" /boot/config.txt; then
    echo "enable_uart=1" >> /boot/config.txt
fi
if ! grep -q "dtoverlay=disable-bt" /boot/config.txt; then
    echo "dtoverlay=disable-bt" >> /boot/config.txt
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
else
    print_info "A reboot is recommended but not required."
fi

print_info "After reboot:"
print_info "1. MAVLink Router will run automatically"
print_info "2. WebGCS will be available at http://[raspberry-pi-ip]:5000"
print_info "3. UART will be configured for the flight controller"
echo
print_info "To reboot now, type: sudo reboot"
