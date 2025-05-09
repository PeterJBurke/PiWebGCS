# PiWebGCS
Web ground control station running on Raspberry Pi.

## Quick Installation

To install PiWebGCS on your Raspberry Pi Zero W, copy and paste these commands into your terminal:

```bash
# 1. Download the installation script
curl -O https://raw.githubusercontent.com/PeterJBurke/PiWebGCS/main/install_drone_control_pi.sh

# 2. Make the script executable
chmod +x install_drone_control_pi.sh

# 3. Run the installation script
sudo ./install_drone_control_pi.sh

# 4. Reboot the system
sudo reboot
```

## What the Installation Does

1. Installs MAVLink Router from https://github.com/PeterJBurke/installmavlinkrouter2024
2. Installs WebGCS from https://github.com/PeterJBurke/WebGCS
3. Sets up Python virtual environment with all dependencies
4. Configures system services for automatic startup
5. Sets up UART for flight controller communication

## After Installation

1. The WebGCS interface will be available at: `http://<raspberry-pi-ip>:5000`
2. MAVLink Router will automatically connect to your flight controller via UART
3. All services will start automatically on boot

## Requirements

- Raspberry Pi Zero W (or other Raspberry Pi)
- Fresh install of Raspberry Pi OS (Raspbian)
- Internet connection for installation
- Flight controller connected to UART (GPIO 14/15)
