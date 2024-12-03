# server.py
import serial.tools.list_ports
from flask import Flask, render_template, request, redirect, url_for, session
import json
import os
import subprocess
import random
import yaml  # For YAML parsing
from pathlib import Path
from functools import wraps

app = Flask(__name__)
app.secret_key = 'your_secret_key_here'  # Replace with a secure random key

# Correctly resolve the paths
BASE_DIR = Path(__file__).resolve().parent
CONFIG_FILE = BASE_DIR / 'config.json'
NETPLAN_FILE = '/etc/netplan/01-network-manager-all.yaml'

# Password for accessing network settings
NETWORK_SETTINGS_PASSWORD = 'boat'  # Replace with your desired password

def login_required(f):
    @wraps(f)
    def decorated_function(*args, **kwargs):
        if 'authenticated' in session and session['authenticated']:
            return f(*args, **kwargs)
        else:
            return redirect(url_for('login', next=request.url))
    return decorated_function

def get_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def load_config():
    try:
        with open(CONFIG_FILE, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        # Return default config if file doesn't exist
        return {'boat_name': '', 'port': ''}
    except json.JSONDecodeError:
        # Handle JSON parsing errors
        return {'boat_name': '', 'port': ''}

def save_config(config):
    with open(CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=4)

def random_mac():
    mac = [0x02, 0x00, 0x00,
           random.randint(0x00, 0x7f),
           random.randint(0x00, 0xff),
           random.randint(0x00, 0xff)]
    return ':'.join(map(lambda x: "%02x" % x, mac))

def update_netplan(mac_address, ssid, wifi_password):
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

@app.route('/', methods=['GET', 'POST'])
def index():
    config = load_config()
    if request.method == 'POST':
        # Handle form data
        boat_name = request.form.get('boat_name', '')
        selected_port = request.form.get('port', '')

        # Update config.json
        config['boat_name'] = boat_name
        config['port'] = selected_port
        save_config(config)

        return redirect(url_for('index'))
    else:
        ports = get_serial_ports()
        current_port = config.get('port', '')
        boat_name = config.get('boat_name', '')

        return render_template('index.html', ports=ports, current_port=current_port,
                               boat_name=boat_name)

@app.route('/network_settings', methods=['GET', 'POST'])
@login_required
def network_settings():
    if request.method == 'POST':
        # Handle form data
        mac_address = request.form.get('mac_address', '')
        ssid = request.form.get('ssid', '')
        wifi_password = request.form.get('wifi_password', '')

        if 'randomize_mac' in request.form:
            mac_address = random_mac()

        # Update netplan configuration
        update_netplan(mac_address, ssid, wifi_password)

        return redirect(url_for('network_settings'))
    else:
        mac_address, ssid, _ = read_netplan()
        return render_template('network_settings.html', mac_address=mac_address,
                               ssid=ssid)

@app.route('/restart', methods=['POST'])
@login_required
def restart():
    # Restart the Raspberry Pi
    try:
        subprocess.run(['sudo', 'reboot'], check=True)
        return 'Restarting...'
    except Exception as e:
        print(f"Error restarting: {e}")
        return 'Failed to restart.'


@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        password = request.form.get('password', '')
        if password == NETWORK_SETTINGS_PASSWORD:
            session['authenticated'] = True
            next_url = request.args.get('next') or url_for('network_settings')
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

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000)
