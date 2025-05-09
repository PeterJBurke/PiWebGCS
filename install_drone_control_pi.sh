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

# Disable Bluetooth to free up UART
echo "dtoverlay=disable-bt" | sudo tee -a /boot/config.txt

# Enable UART
echo "enable_uart=1" | sudo tee -a /boot/config.txt

# Stop and disable Bluetooth services
systemctl stop bluetooth
systemctl disable bluetooth

# Create serial device if it doesn't exist
if [ ! -e "/dev/serial0" ]; then
    ln -s /dev/ttyAMA0 /dev/serial0
fi

# Ensure proper permissions
chmod 666 /dev/ttyAMA0
chmod 666 /dev/serial0

# --- 4. Install Raspberry Pi Hotspot Failover ---
print_info "Installing Raspberry Pi Hotspot Failover..."
cd /home/pi
if [ -d "RaspberryPiHotspotIfNoWifi" ]; then
    rm -rf "RaspberryPiHotspotIfNoWifi"
fi
git clone https://github.com/PeterJBurke/RaspberryPiHotspotIfNoWifi.git
cd RaspberryPiHotspotIfNoWifi
chmod +x install.sh
./install.sh

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
print_info "Please reboot the system."
print_info "After reboot:"
print_info "1. MAVLink Router will run automatically"
print_info "2. WebGCS will be available at http://[raspberry-pi-ip]:5000"
print_info "3. UART will be configured for the flight controller"
echo
print_info "To reboot now, type: sudo reboot"
