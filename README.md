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

1. Configures UART for flight controller communication
   - Disables Bluetooth to free up UART
   - Enables UART in boot config
   - Sets up proper permissions
2. Installs MAVLink Router from https://github.com/PeterJBurke/installmavlinkrouter2024
3. Installs WebGCS from https://github.com/PeterJBurke/WebGCS
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

## Troubleshooting Guide

### Checking Service Status

Check the status of all related services:
```bash
# Check MAVLink Router service
sudo systemctl status mavlink-router

# Check WebGCS service
sudo systemctl status webgcs

# Check WiFi Hotspot service
sudo systemctl status create_ap
```

### Viewing Log Files

1. View logs in real-time (continuous):
```bash
# MAVLink Router logs
sudo journalctl -fu mavlink-router

# WebGCS logs
sudo journalctl -fu webgcs

# WiFi Hotspot logs
sudo journalctl -fu create_ap
```

2. View recent log snapshots:
```bash
# Last 50 lines of MAVLink Router logs
sudo journalctl -n 50 -u mavlink-router

# Last 50 lines of WebGCS logs
sudo journalctl -n 50 -u webgcs

# Last 50 lines of WiFi Hotspot logs
sudo journalctl -n 50 -u create_ap
```

### Common Issues and Solutions

1. **WebGCS not accessible:**
   - Check if the service is running: `sudo systemctl status webgcs`
   - Verify your Pi's IP address: `hostname -I`
   - Test local access: `curl http://localhost:5000`
   - Check firewall settings: `sudo ufw status`

2. **No connection to flight controller:**
   - Check if UART is properly configured:
     ```bash
     # Check if UART is enabled in boot config
     grep "enable_uart" /boot/config.txt
     grep "dtoverlay=disable-bt" /boot/config.txt
     
     # Verify serial device exists
     ls -l /dev/serial0
     ls -l /dev/ttyAMA0
     
     # Check permissions
     sudo chmod 666 /dev/ttyAMA0
     sudo chmod 666 /dev/serial0
     ```
   - Check if user 'pi' is in dialout group: `groups pi`
   - Verify Bluetooth is disabled:
     ```bash
     systemctl status bluetooth
     hciconfig
     ```
   - Check physical connections and wiring
   - Verify baud rate matches flight controller (default: 57600)
   - If issues persist, try rebooting: `sudo reboot`

3. **WiFi Hotspot issues:**
   - Check if hostapd is installed: `dpkg -l | grep hostapd`
   - Verify wireless interface: `iwconfig`
   - Check create_ap service status: `sudo systemctl status create_ap`
   - View WiFi debugging info: `sudo iw dev`

4. **Performance issues:**
   - Check CPU usage: `top`
   - Monitor temperature: `vcgencmd measure_temp`
   - Check disk space: `df -h`
   - View memory usage: `free -h`

### Manual Service Control

Restart services:
```bash
sudo systemctl restart mavlink-router
sudo systemctl restart webgcs
sudo systemctl restart create_ap
```

Stop services:
```bash
sudo systemctl stop mavlink-router
sudo systemctl stop webgcs
sudo systemctl stop create_ap
```

Start services:
```bash
sudo systemctl start mavlink-router
sudo systemctl start webgcs
sudo systemctl start create_ap
```

### Additional Tips

1. **Network Connectivity:**
   - Test internet connectivity: `ping 8.8.8.8`
   - Check network interfaces: `ifconfig`
   - View network routing: `netstat -rn`

2. **System Health:**
   - Check system logs: `sudo dmesg | tail`
   - View boot logs: `journalctl -b`
   - Monitor resource usage: `htop` (install if needed)

3. **Configuration Files:**
   - MAVLink Router config: `/etc/mavlink-router/main.conf`
   - WebGCS config: `/home/pi/WebGCS/config.py`
   - WiFi Hotspot config: `/etc/create_ap.conf`

If issues persist after trying these solutions, check the GitHub issues page or create a new issue with:
- Relevant log outputs
- System information (`uname -a`)
- Description of the problem
- Steps to reproduce
