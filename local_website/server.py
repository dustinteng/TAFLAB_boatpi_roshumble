# server.py
import serial.tools.list_ports
from flask import Flask, render_template, request, redirect, url_for, session
import json
import os
import subprocess
import random
import yaml  # For YAML parsing
from pathlib import Path
import re
from functools import wraps

app = Flask(__name__)
app.secret_key = 'your_secret_key_here'  # Replace with a secure random key

# Correctly resolve the paths
BASE_DIR = Path(__file__).resolve().parent
CONFIG_FILE = BASE_DIR / 'config.json'
NETPLAN_FILE = '/etc/netplan/01-network-manager-all.yaml'
HOSTAPD_CONF = '/etc/hostapd/hostapd.conf'

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
    
def get_current_mac(interface='wlan0'):
    """Get the current MAC address of a network interface."""
    try:
        # Use the `ip` command to fetch interface details
        result = subprocess.check_output(['ip', 'link', 'show', interface], text=True)
        # Regex to find the MAC address in the command output
        mac_address = re.search(r"link/ether ([0-9a-f:]{17})", result).group(1)
        return mac_address
    except subprocess.CalledProcessError as e:
        print(f"Error running 'ip' command: {e}")
    except AttributeError:
        print(f"Could not find MAC address for interface {interface}.")
    return "Unknown"

def get_current_ssid():
    """Get the currently connected Wi-Fi SSID."""
    try:
        result = subprocess.check_output(['nmcli', '-t', '-f', 'ACTIVE,SSID', 'dev', 'wifi'], text=True)
        for line in result.splitlines():
            if line.startswith('yes:'):
                return line.split(':')[1]  # Return the SSID after 'yes:'
    except subprocess.CalledProcessError as e:
        print(f"Error getting current SSID: {e}")
    return "Unknown"

def get_available_networks():
    """Get a list of available Wi-Fi SSIDs."""
    try:
        result = subprocess.check_output(['nmcli', '-t', '-f', 'SSID', 'dev', 'wifi'], text=True)
        ssids = list(filter(None, result.splitlines()))  # Remove empty lines
        return sorted(set(ssids))  # Remove duplicates and sort
    except subprocess.CalledProcessError as e:
        print(f"Error getting available networks: {e}")
    return []

def get_ap_status():
    """Check if the wlan0_ap interface is active and retrieve the SSID."""
    try:
        # Check if wlan0_ap is active
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
    """Update the /etc/hostapd/hostapd.conf file with new SSID and password."""
    try:
        # Read existing hostapd.conf content
        with open('/etc/hostapd/hostapd.conf', 'r') as f:
            lines = f.readlines()
        
        # Modify the SSID and password lines
        updated_lines = []
        for line in lines:
            if line.startswith('ssid='):
                updated_lines.append(f'ssid={new_ssid}\n')
            elif line.startswith('wpa_passphrase='):
                updated_lines.append(f'wpa_passphrase={new_password}\n')
            else:
                updated_lines.append(line)

        # Write the updated lines back to the file
        with open('/etc/hostapd/hostapd.conf', 'w') as f:
            f.writelines(updated_lines)
        
        # Restart the hostapd service to apply changes
        subprocess.run(['sudo', 'systemctl', 'restart', 'hostapd'], check=True)
    except Exception as e:
        print(f"Error updating hostapd configuration: {e}")


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

        ap_active, ap_ssid = get_ap_status()

        return render_template('index.html', ports=ports, current_port=current_port,
                               boat_name=boat_name, ap_active=ap_active, ap_ssid=ap_ssid)

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
        current_mac = get_current_mac()  # Dynamically get the MAC address
        available_networks = get_available_networks()
        connected_ssid = get_current_ssid()
        return render_template(
            'network_settings.html',
            mac_address=current_mac,
            connected_ssid=connected_ssid,
            available_networks=available_networks
        )

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

@app.route('/refresh_mac', methods=['GET'])
def refresh_mac():
    """API endpoint to refresh the current MAC address."""
    current_mac = get_current_mac()  # Dynamically get the MAC address
    return {"mac_address": current_mac}, 200

@app.route('/ap_settings', methods=['GET', 'POST'])
@login_required
def ap_settings():
    if request.method == 'POST':
        # Get form data
        new_ssid = request.form.get('ap_ssid', '').strip()
        new_password = request.form.get('ap_password', '').strip()

        if not new_ssid or not new_password:
            return render_template('ap_settings.html', error="SSID and Password cannot be empty.")

        # Update the hostapd configuration
        update_hostapd_config(new_ssid, new_password)

        return render_template('ap_settings.html', success="AP settings updated successfully!")

    else:
        # Fetch current SSID and Password from hostapd.conf
        current_ssid, current_password = '', ''
        try:
            with open('/etc/hostapd/hostapd.conf', 'r') as f:
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
