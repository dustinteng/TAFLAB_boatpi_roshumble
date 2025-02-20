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
# import smbus2
# from mpu9250_jmdev.registers import *
# from mpu9250_jmdev.mpu_9250 import MPU9250

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
    """
    Loads the configuration from CONFIG_FILE.
    If the file doesn't exist or is invalid, returns a default config that now
    includes magnetometer calibration settings.
    """
    default_config = {
        'boat_name': '',
        'xbee_port': '',
        'mag_port' : '',
        'mag_offset': [-3.78, 4.096, 2.163],
        'mag_matrix': [
            [1.00, 0.0078, -0.011],
            [0.0078, 0.979, 0.0669],
            [-0.0110, 0.0669, 1.0453]
        ],
        'heading_offset': 0.0
    }
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
        # Ensure new keys are present; if not, add default values.
        for key, value in default_config.items():
            if key not in config:
                config[key] = value
        return config
    except (FileNotFoundError, json.JSONDecodeError):
        return default_config

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
                return line.split(':')[1]
    except subprocess.CalledProcessError as e:
        print(f"Error getting current SSID: {e}")
    return "Unknown"

def get_available_networks():
    """Use nmcli to retrieve a list of available Wi-Fi SSIDs."""
    try:
        result = subprocess.check_output(['nmcli', '-t', '-f', 'SSID', 'dev', 'wifi'], text=True)
        ssids = list(filter(None, result.splitlines()))
        return sorted(set(ssids))
    except subprocess.CalledProcessError as e:
        print(f"Error getting available networks: {e}")
    return []

def get_ap_status():
    """Check if the wlan0_ap interface is active and retrieve the SSID from hostapd.conf."""
    try:
        result = subprocess.check_output(['ip', 'addr', 'show', 'wlan0_ap'], text=True)
        if 'state UP' in result:
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
        with open(HOSTAPD_CONF, 'r') as f:
            lines = f.readlines()

        updated_lines = []
        for line in lines:
            if line.startswith('ssid='):
                updated_lines.append(f'ssid={new_ssid}\n')
            elif line.startswith('wpa_passphrase='):
                updated_lines.append(f'wpa_passphrase={new_password}\n')
            else:
                updated_lines.append(line)

        with open(temp_file, 'w') as f:
            f.writelines(updated_lines)

        result = subprocess.run(['sudo', 'mv', temp_file, HOSTAPD_CONF], check=False)
        if result.returncode != 0:
            print("Failed to replace the hostapd.conf file.")
            return False

        with open(HOSTAPD_CONF, 'r') as f:
            updated_content = f.read()
            if f"ssid={new_ssid}" not in updated_content or f"wpa_passphrase={new_password}" not in updated_content:
                print("Error: Changes were not successfully written to hostapd.conf.")
                return False

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
        if os.path.exists(temp_file):
            os.remove(temp_file)
    return False

###############################################################################
#                               FLASK ROUTES
###############################################################################
@app.route('/', methods=['GET', 'POST'])
def index():
    """Main index page for boat name, XBee port, and Magnetometer port selection."""
    config = load_config()
    if request.method == 'POST':
        boat_name = request.form.get('boat_name', '')
        selected_port = request.form.get('xbee_port', '')
        mag_port = request.form.get('mag_port', '')  # Retrieve mag_port from form
        ap_ssid = request.form.get('ap_ssid', '').strip()
        ap_password = request.form.get('ap_password', '').strip()

        config['boat_name'] = boat_name
        config['xbee_port'] = selected_port
        config['mag_port'] = mag_port  # Save mag_port in config
        save_config(config)

        if ap_ssid and ap_password:
            update_hostapd_config(ap_ssid, ap_password)

        return redirect(url_for('index'))
    else:
        ports = get_serial_ports()
        xbee_port = config.get('xbee_port', '')
        boat_name = config.get('boat_name', '')
        mag_port = config.get('mag_port', '')  # Get mag_port from config
        ap_active, ap_ssid = get_ap_status()

        return render_template(
            'index.html',
            ports=ports,
            xbee_port=xbee_port,
            boat_name=boat_name,
            mag_port=mag_port,
            ap_active=ap_active,
            ap_ssid=ap_ssid
        )

@app.route('/admin', methods=['GET', 'POST'])
@login_required
def admin():
    """
    Admin page for network settings (Wi-Fi, MAC) and for displaying calibration settings.
    """
    if request.method == 'POST':
        mac_address = request.form.get('mac_address', '')
        ssid = request.form.get('ssid', '')
        wifi_password = request.form.get('wifi_password', '')

        if 'randomize_mac' in request.form:
            mac_address = random_mac()

        update_netplan(mac_address, ssid, wifi_password)
        return redirect(url_for('admin'))
    else:
        current_mac = get_current_mac()
        available_networks = get_available_networks()
        connected_ssid = get_current_ssid()
        config = load_config()

        return render_template(
            'admin.html',
            mac_address=current_mac,
            connected_ssid=connected_ssid,
            available_networks=available_networks,
            config=config
        )

@app.route('/update_calibration_config', methods=['POST'])
@login_required
def update_calibration_config():
    """
    Route to update calibration configuration values from the admin form.
    """
    config = load_config()
    try:
        # Retrieve calibration values from the form and convert to floats.
        mag_offset = [
            float(request.form.get('mag_offset_0', config.get('mag_offset', [0, 0, 0])[0])),
            float(request.form.get('mag_offset_1', config.get('mag_offset', [0, 0, 0])[1])),
            float(request.form.get('mag_offset_2', config.get('mag_offset', [0, 0, 0])[2]))
        ]
        mag_matrix = [
            [
                float(request.form.get('mag_matrix_0_0', config.get('mag_matrix', [[0,0,0],[0,0,0],[0,0,0]])[0][0])),
                float(request.form.get('mag_matrix_0_1', config.get('mag_matrix', [[0,0,0],[0,0,0],[0,0,0]])[0][1])),
                float(request.form.get('mag_matrix_0_2', config.get('mag_matrix', [[0,0,0],[0,0,0],[0,0,0]])[0][2]))
            ],
            [
                float(request.form.get('mag_matrix_1_0', config.get('mag_matrix', [[0,0,0],[0,0,0],[0,0,0]])[1][0])),
                float(request.form.get('mag_matrix_1_1', config.get('mag_matrix', [[0,0,0],[0,0,0],[0,0,0]])[1][1])),
                float(request.form.get('mag_matrix_1_2', config.get('mag_matrix', [[0,0,0],[0,0,0],[0,0,0]])[1][2]))
            ],
            [
                float(request.form.get('mag_matrix_2_0', config.get('mag_matrix', [[0,0,0],[0,0,0],[0,0,0]])[2][0])),
                float(request.form.get('mag_matrix_2_1', config.get('mag_matrix', [[0,0,0],[0,0,0],[0,0,0]])[2][1])),
                float(request.form.get('mag_matrix_2_2', config.get('mag_matrix', [[0,0,0],[0,0,0],[0,0,0]])[2][2]))
            ]
        ]
        heading_offset = float(request.form.get('heading_offset', config.get('heading_offset', 0.0)))
    except ValueError:
        return "Invalid input: Please ensure all calibration values are numeric.", 400

    config['mag_offset'] = mag_offset
    config['mag_matrix'] = mag_matrix
    config['heading_offset'] = heading_offset
    save_config(config)
    return redirect(url_for('admin'))

@app.route('/refresh_mac', methods=['GET'])
def refresh_mac():
    """API endpoint to refresh the current MAC address."""
    current_mac = get_current_mac()
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

        print(f"Received new SSID: {new_ssid}")
        print(f"Received new Password: {new_password}")

        success = update_hostapd_config(new_ssid, new_password)
        if success:
            return render_template('ap_settings.html', success="AP settings updated successfully!")
        else:
            return render_template('ap_settings.html', error="Failed to update AP settings.")
    else:
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
    """Simple password-based login."""
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
        bus.write_byte_data(0x68, 0x37, 0x02)
        bus.write_byte_data(0x68, 0x24, 0x22)
        print("I2C bypass mode enabled on MPU9250.")
        bus.close()
    except Exception as e:
        print(f"Error enabling I2C bypass: {e}")

# Enable bypass before initializing MPU9250
enable_i2c_bypass()

mpu = MPU9250(
    address_ak=0x0C,
    address_mpu_master=0x68,
    address_mpu_slave=None,
    bus=1,
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
        mpu.calibrate()
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
    app.run(host='0.0.0.0', port=3333)
