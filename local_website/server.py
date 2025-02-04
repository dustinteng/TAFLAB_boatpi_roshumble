###############################################################################
#                             IMPORTS & SETUP
###############################################################################
import serial.tools.list_ports
from flask import Flask, render_template, request, redirect, url_for, session, jsonify
import json
import os
import subprocess
import random
import yaml  # For YAML parsing
from pathlib import Path
import re
from functools import wraps

# For MPU and I2C bypass
import smbus2
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

app = Flask(__name__)
app.secret_key = 'your_secret_key_here'  # Replace with a secure random key

RESTART_SCRIPT = '/home/boat/Desktop/python/TAFLAB_boatpi_roshumble/local_website/restart.sh'
CONFIG_FILE = Path("/home/boat/Desktop/python/TAFLAB_boatpi_roshumble/src/config.json")
NETPLAN_FILE = '/etc/netplan/01-network-manager-all.yaml'
HOSTAPD_CONF = '/etc/hostapd/hostapd.conf'
NETWORK_SETTINGS_PASSWORD = 'boat'  # Replace with your desired password

###############################################################################
#                          LOGIN & DECORATORS
###############################################################################
def login_required(f):
    @wraps(f)
    def decorated_function(*args, **kwargs):
        if 'authenticated' in session and session['authenticated']:
            return f(*args, **kwargs)
        else:
            return redirect(url_for('login', next=request.url))
    return decorated_function


###############################################################################
#                             HELPER FUNCTIONS
###############################################################################
def get_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def load_config():
    try:
        with open(CONFIG_FILE, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        # Return default config if file doesn't exist
        return {'boat_name': '', 'xbee_port': ''}
    except json.JSONDecodeError:
        # Handle JSON parsing errors
        return {'boat_name': '', 'xbee_port': ''}

def save_config(config):
    with open(CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=4)

def random_mac():
    """Generate a semi-random MAC address (local/admin)."""
    mac = [
        0x02, 0x00, 0x00,  # Locally administered MAC address
        random.randint(0x00, 0x7f),
        random.randint(0x00, 0xff),
        random.randint(0x00, 0xff)
    ]
    return ':'.join(map(lambda x: "%02x" % x, mac))

def update_netplan(mac_address, ssid, wifi_password):
    """Write a new netplan config and apply it."""
    netplan_config = {
        'network': {
            'version': 2,
            'renderer': 'NetworkManager',
            'wifis': {
                'wlan0': {
                    'dhcp4': True,
                    'macaddress': mac_address,
                    'access-points': {
                        ssid: {
                            'password': wifi_password
                        }
                    }
                }
            }
        }
    }
    try:
        with open(NETPLAN_FILE, 'w') as f:
            yaml.dump(netplan_config, f, default_flow_style=False)
        # Apply netplan changes
        subprocess.run(['sudo', 'netplan', 'apply'], check=True)
    except Exception as e:
        print(f"Error updating netplan: {e}")

def read_netplan():
    """Read netplan YAML to retrieve MAC, SSID, and password (if any)."""
    if os.path.exists(NETPLAN_FILE):
        try:
            with open(NETPLAN_FILE, 'r') as f:
                netplan_content = yaml.safe_load(f)
            wlan0_config = netplan_content.get('network', {}).get('wifis', {}).get('wlan0', {})
            mac_address = wlan0_config.get('macaddress', '')
            ssid = next(iter(wlan0_config.get('access-points', {})), '')
            wifi_password = wlan0_config.get('access-points', {}).get(ssid, {}).get('password', '')
            return mac_address, ssid, wifi_password
        except Exception as e:
            print(f"Error reading netplan: {e}")
            return '', '', ''
    else:
        return '', '', ''
    
def get_current_mac(interface='wlan0'):
    """Get the current MAC address of a network interface."""
    try:
        result = subprocess.check_output(['ip', 'link', 'show', interface], text=True)
        mac_address = re.search(r"link/ether ([0-9a-f:]{17})", result).group(1)
        return mac_address
    except subprocess.CalledProcessError as e:
        print(f"Error running 'ip' command: {e}")
    except AttributeError:
        print(f"Could not find MAC address for interface {interface}.")
    return "Unknown"

def get_current_ssid():
    """Get the currently connected Wi-Fi SSID using nmcli."""
    try:
        result = subprocess.check_output(['nmcli', '-t', '-f', 'ACTIVE,SSID', 'dev', 'wifi'], text=True)
        for line in result.splitlines():
            if line.startswith('yes:'):
                return line.split(':')[1]  # Return the SSID after 'yes:'
    except subprocess.CalledProcessError as e:
        print(f"Error getting current SSID: {e}")
    return "Unknown"

def get_available_networks():
    """Use nmcli to retrieve a list of available Wi-Fi SSIDs."""
    try:
        result = subprocess.check_output(['nmcli', '-t', '-f', 'SSID', 'dev', 'wifi'], text=True)
        ssids = list(filter(None, result.splitlines()))  # Remove empty lines
        return sorted(set(ssids))  # Remove duplicates and sort
    except subprocess.CalledProcessError as e:
        print(f"Error getting available networks: {e}")
    return []

def get_ap_status():
    """Check if the wlan0_ap interface is active and retrieve the SSID from hostapd.conf."""
    try:
        result = subprocess.check_output(['ip', 'addr', 'show', 'wlan0_ap'], text=True)
        if 'state UP' in result:
            # Read the SSID from the hostapd configuration
            ssid = ''
            with open(HOSTAPD_CONF, 'r') as f:
                for line in f:
                    if line.startswith('ssid='):
                        ssid = line.split('=')[1].strip()
                        break
            return True, ssid
        return False, ''
    except subprocess.CalledProcessError as e:
        print(f"Error checking wlan0_ap status: {e}")
        return False, ''
    
def update_hostapd_config(new_ssid, new_password):
    """
    Update the /etc/hostapd/hostapd.conf file with a new SSID and password,
    then restart hostapd to apply changes.
    """
    temp_file = '/tmp/hostapd_temp.conf'
    try:
        # Step 1: Read existing hostapd.conf content
        with open(HOSTAPD_CONF, 'r') as f:
            lines = f.readlines()

        # Step 2: Modify the SSID and password lines
        updated_lines = []
        for line in lines:
            if line.startswith('ssid='):
                updated_lines.append(f'ssid={new_ssid}\n')
            elif line.startswith('wpa_passphrase='):
                updated_lines.append(f'wpa_passphrase={new_password}\n')
            else:
                updated_lines.append(line)

        # Step 3: Write the updated lines to a temporary file
        with open(temp_file, 'w') as f:
            f.writelines(updated_lines)

        # Step 4: Move the temp file to hostapd.conf
        result = subprocess.run(['sudo', 'mv', temp_file, HOSTAPD_CONF], check=False)
        if result.returncode != 0:
            print("Failed to replace the hostapd.conf file.")
            return False

        # Step 5: Re-read the updated configuration to confirm the changes
        with open(HOSTAPD_CONF, 'r') as f:
            updated_content = f.read()
            if f"ssid={new_ssid}" not in updated_content or f"wpa_passphrase={new_password}" not in updated_content:
                print("Error: Changes were not successfully written to hostapd.conf.")
                return False

        # Step 6: Restart the hostapd service to apply changes
        subprocess.run(['sudo', 'systemctl', 'restart', 'hostapd'], check=True)
        print(f"Successfully updated {HOSTAPD_CONF} with SSID '{new_ssid}' and restarted hostapd.")
        return True

    except FileNotFoundError:
        print(f"Error: {HOSTAPD_CONF} not found.")
    except PermissionError:
        print(f"Error: Permission denied when writing to {HOSTAPD_CONF}.")
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # Step 7: Clean up temporary file if it still exists
        if os.path.exists(temp_file):
            os.remove(temp_file)
    return False  # If we reached here, it didn't succeed fully


###############################################################################
#                               FLASK ROUTES
###############################################################################
@app.route('/', methods=['GET', 'POST'])
def index():
    """Main index page for boat name and XBee port selection."""
    config = load_config()
    if request.method == 'POST':
        # Handle form data
        boat_name = request.form.get('boat_name', '')
        selected_port = request.form.get('xbee_port', '')
        ap_ssid = request.form.get('ap_ssid', '').strip()
        ap_password = request.form.get('ap_password', '').strip()

        # Update config.json
        config['boat_name'] = boat_name
        config['xbee_port'] = selected_port
        save_config(config)

        # Update AP settings (if provided)
        if ap_ssid and ap_password:
            update_hostapd_config(ap_ssid, ap_password)

        return redirect(url_for('index'))
    else:
        # Render the form
        ports = get_serial_ports()
        current_port = config.get('xbee_port', '')
        boat_name = config.get('boat_name', '')
        ap_active, ap_ssid = get_ap_status()

        return render_template(
            'index.html',
            ports=ports,
            current_port=current_port,
            boat_name=boat_name,
            ap_active=ap_active,
            ap_ssid=ap_ssid
        )

@app.route('/admin', methods=['GET', 'POST'])
@login_required
def admin():
    """
    Admin page for network settings (Wi-Fi, MAC).
    """
    if request.method == 'POST':
        # Handle form data
        mac_address = request.form.get('mac_address', '')
        ssid = request.form.get('ssid', '')
        wifi_password = request.form.get('wifi_password', '')

        # If 'randomize_mac' was checked, generate a new MAC
        if 'randomize_mac' in request.form:
            mac_address = random_mac()

        # Update netplan configuration
        update_netplan(mac_address, ssid, wifi_password)

        return redirect(url_for('admin'))
    else:
        current_mac = get_current_mac()  # Dynamically get the MAC address
        available_networks = get_available_networks()
        connected_ssid = get_current_ssid()
        return render_template(
            'admin.html',
            mac_address=current_mac,
            connected_ssid=connected_ssid,
            available_networks=available_networks
        )

@app.route('/refresh_mac', methods=['GET'])
def refresh_mac():
    """API endpoint to refresh the current MAC address."""
    current_mac = get_current_mac()  # Dynamically get the MAC address
    return {"mac_address": current_mac}, 200

@app.route('/restart', methods=['POST'])
@login_required
def restart():
    """Restart the Raspberry Pi."""
    try:
        subprocess.run(['sudo', RESTART_SCRIPT], check=True)
        return 'Restarting...'
    except Exception as e:
        print(f"Error restarting: {e}")
        return 'Failed to restart.', 500

@app.route('/ap_settings', methods=['GET', 'POST'])
@login_required
def ap_settings():
    """Form to update Access Point SSID/Password."""
    if request.method == 'POST':
        new_ssid = request.form.get('ap_ssid', '').strip()
        new_password = request.form.get('ap_password', '').strip()

        if not new_ssid or not new_password:
            return render_template('ap_settings.html', error="SSID and Password cannot be empty.")

        # Debugging: Print values received
        print(f"Received new SSID: {new_ssid}")
        print(f"Received new Password: {new_password}")

        success = update_hostapd_config(new_ssid, new_password)
        if success:
            return render_template('ap_settings.html', success="AP settings updated successfully!")
        else:
            return render_template('ap_settings.html', error="Failed to update AP settings.")
    else:
        # Fetch current SSID and Password from hostapd.conf
        current_ssid, current_password = '', ''
        try:
            with open(HOSTAPD_CONF, 'r') as f:
                for line in f:
                    if line.startswith('ssid='):
                        current_ssid = line.split('=')[1].strip()
                    elif line.startswith('wpa_passphrase='):
                        current_password = line.split('=')[1].strip()
        except Exception as e:
            print(f"Error reading hostapd configuration: {e}")

        return render_template('ap_settings.html', current_ssid=current_ssid, current_password=current_password)

@app.route('/login', methods=['GET', 'POST'])
def login():
    """Simple password-based login (replace with more secure auth in production)."""
    if request.method == 'POST':
        password = request.form.get('password', '')
        if password == NETWORK_SETTINGS_PASSWORD:
            session['authenticated'] = True
            next_url = request.args.get('next') or url_for('admin')
            return redirect(next_url)
        else:
            error = 'Invalid password'
            return render_template('login.html', error=error)
    else:
        return render_template('login.html')

@app.route('/logout')
def logout():
    session.pop('authenticated', None)
    return redirect(url_for('index'))


###############################################################################
#                          I2C BYPASS & MPU9250 SETUP
###############################################################################
@app.route('/enable_i2c_bypass', methods=['POST'])
def enable_i2c_bypass():
    """
    Enable I2C bypass on the MPU9250 so the AK8963 magnetometer (0x0C)
    can be accessed directly on the bus.
    """
    try:
        bus = smbus2.SMBus(1)
        # Register 0x37: INT_PIN_CFG -> BYPASS_EN = 0x02
        bus.write_byte_data(0x68, 0x37, 0x02)
        # Register 0x24: I2C_MST_CTRL. 0x22 => for example 400KHz. Adjust if needed.
        bus.write_byte_data(0x68, 0x24, 0x22)
        print("I2C bypass mode enabled on MPU9250.")
        bus.close()
    except Exception as e:
        print(f"Error enabling I2C bypass: {e}")


enable_i2c_bypass()  # Enable bypass before initializing MPU9250

# Now initialize MPU9250
mpu = MPU9250(
    address_ak=0x0C,  # AK8963 magnetometer address
    address_mpu_master=0x68,  # MPU9250 address
    address_mpu_slave=None,
    bus=1,  # I2C bus (likely /dev/i2c-1)
    gfs=GFS_250,
    afs=AFS_2G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ
)


###############################################################################
#                           CALIBRATION LOGIC
###############################################################################
CALIBRATION_FILE = Path("/home/boat/Desktop/python/TAFLAB_boatpi_roshumble/src/mpu_calibration.json")

def save_calibration():
    """Saves calibration data to a file."""
    calibration_data = {
        "abias": mpu.abias,
        "gbias": mpu.gbias,
        "mbias": mpu.mbias
    }
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(calibration_data, f, indent=4)
    print("Calibration saved.")

def load_calibration():
    """Loads calibration data from file."""
    try:
        with open(CALIBRATION_FILE, "r") as f:
            calibration_data = json.load(f)
            mpu.abias = calibration_data.get("abias", mpu.abias)
            mpu.gbias = calibration_data.get("gbias", mpu.gbias)
            mpu.mbias = calibration_data.get("mbias", mpu.mbias)
        print("Calibration loaded successfully.")
    except FileNotFoundError:
        print("No calibration file found. Using default values.")
    except json.JSONDecodeError as e:
        print(f"Error parsing calibration file: {e}")

# Load calibration at startup (after creating `mpu`)
load_calibration()

@app.route('/calibrate_imu', methods=['POST'])
@login_required
def calibrate_imu():
    """
    Calibrates the accelerometer and gyroscope (IMU).
    Keep the sensor still during this calibration.
    """
    try:
        print("Calibrating Accelerometer and Gyroscope... Keep sensor still.")
        mpu.calibrate()  # Calibrate IMU
        save_calibration()
        return {"status": "success", "message": "IMU calibration complete."}, 200
    except Exception as e:
        return {"status": "error", "message": f"IMU calibration failed: {e}"}, 500

@app.route('/calibrate_magnetometer', methods=['POST'])
@login_required
def calibrate_magnetometer():
    """
    Calibrates the magnetometer.
    Move the sensor in a figure-eight pattern.
    """
    try:
        print("Calibrating Magnetometer... Move the sensor in a figure-eight pattern.")
        mpu.calibrateAK8963()
        save_calibration()
        return {"status": "success", "message": "Magnetometer calibration complete."}, 200
    except Exception as e:
        return {"status": "error", "message": f"Magnetometer calibration failed: {e}"}, 500


###############################################################################
#                           RUN THE FLASK APP
###############################################################################
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=3334)
