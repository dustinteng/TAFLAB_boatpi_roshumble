#!/usr/bin/env python3
from flask import Flask, render_template_string, request, jsonify
import pigpio
from smbus2 import SMBus
import threading
import time

# Define GPIO pins for rudder and sail servos
RUDDER_PIN = 18
SAIL_PIN = 12

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    raise Exception("Could not connect to pigpio daemon")

# Set up GPIO pins and PWM frequency for rudder and sail
pi.set_mode(RUDDER_PIN, pigpio.OUTPUT)
pi.set_PWM_frequency(RUDDER_PIN, 50)
pi.set_mode(SAIL_PIN, pigpio.OUTPUT)
pi.set_PWM_frequency(SAIL_PIN, 50)

# Global configuration (duty cycle in µs) for rudder and sail
rudder_config = {"min": 500, "mid": 1500, "max": 2500}
sail_config = {"min": 500, "mid": 1500, "max": 2500}

# Windvane sensor configuration
AS5600_ADDRESS = 0x36
RAW_ANGLE_REGISTER = 0x0C
bus = SMBus(1)  # I2C bus for windvane sensor

# Global variables for windvane
windvane_offset = -3231  # initial offset
last_raw_value = 0       # most recent raw sensor reading
windvane_angle = 0       # computed angle (in degrees)
reverse = True           # reverse output if needed

def read_windvane():
    global windvane_angle, last_raw_value, windvane_offset, reverse
    try:
        # Read two bytes from the sensor
        angle_data = bus.read_i2c_block_data(AS5600_ADDRESS, RAW_ANGLE_REGISTER, 2)
        raw_value = (angle_data[0] << 8) | angle_data[1]
        last_raw_value = raw_value  # save raw value for calibration
        # Apply offset and wrap within 0-4096 range
        raw_angle = raw_value + windvane_offset
        if raw_angle < 0:
            raw_angle += 4096
        elif raw_angle > 4096:
            raw_angle -= 4096
        # Convert to degrees (12-bit resolution)
        angle_degrees = (raw_angle / 4096.0) * 360.0
        if reverse:
            angle_degrees = 360 - angle_degrees
        windvane_angle = angle_degrees
    except Exception as e:
        print("Error reading windvane:", e)

def windvane_loop():
    while True:
        read_windvane()
        time.sleep(0.25)  # update at 10 Hz

# Start windvane reading thread (runs in background)
windvane_thread = threading.Thread(target=windvane_loop, daemon=True)
windvane_thread.start()

# HTML template with sections for rudder, sail, and windvane
html_page = """
<!DOCTYPE html>
<html>
<head>
  <title>Servo and Windvane Control</title>
</head>
<body>
  <h1>Servo and Windvane Control</h1>
  
  <!-- Rudder Section -->
  <div>
    <h2>Rudder</h2>
    <div>
      <label>Min Duty:</label>
      <input type="number" id="rudderMin" value="{{ rudder_config.min }}">
      <label>Mid Duty:</label>
      <input type="number" id="rudderMid" value="{{ rudder_config.mid }}">
      <label>Max Duty:</label>
      <input type="number" id="rudderMax" value="{{ rudder_config.max }}">
      <button onclick="updateRudderConfig()">Update Rudder Config</button>
    </div>
    <br>
    <input type="range" id="rudderSlider" min="{{ rudder_config.min }}" max="{{ rudder_config.max }}" value="{{ rudder_config.mid }}" step="1" style="width:80%;">
    <span id="rudderValue">{{ rudder_config.mid }}</span>
  </div>
  
  <hr>
  
  <!-- Sail Section -->
  <div>
    <h2>Sail</h2>
    <div>
      <label>Min Duty:</label>
      <input type="number" id="sailMin" value="{{ sail_config.min }}">
      <label>Mid Duty:</label>
      <input type="number" id="sailMid" value="{{ sail_config.mid }}">
      <label>Max Duty:</label>
      <input type="number" id="sailMax" value="{{ sail_config.max }}">
      <button onclick="updateSailConfig()">Update Sail Config</button>
    </div>
    <br>
    <input type="range" id="sailSlider" min="{{ sail_config.min }}" max="{{ sail_config.max }}" value="{{ sail_config.mid }}" step="1" style="width:80%;">
    <span id="sailValue">{{ sail_config.mid }}</span>
  </div>
  
  <hr>
  
  <!-- Windvane Section -->
  <div>
    <h2>Windvane Sensor</h2>
    <p>Current Angle: <span id="windvaneAngle">0</span> degrees</p>
    <button onclick="calibrateWindvane()">Calibrate Windvane</button>
    <p id="calibrateMsg"></p>
  </div>
  
  <script>
    // Rudder configuration update
    function updateRudderConfig() {
      const min = parseInt(document.getElementById("rudderMin").value);
      const mid = parseInt(document.getElementById("rudderMid").value);
      const max = parseInt(document.getElementById("rudderMax").value);
      
      fetch("/update_rudder_config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ min: min, mid: mid, max: max })
      })
      .then(response => response.json())
      .then(data => {
        const slider = document.getElementById("rudderSlider");
        slider.min = data.min;
        slider.max = data.max;
        slider.value = data.mid;
        document.getElementById("rudderValue").innerText = data.mid;
      });
    }
    
    // Sail configuration update
    function updateSailConfig() {
      const min = parseInt(document.getElementById("sailMin").value);
      const mid = parseInt(document.getElementById("sailMid").value);
      const max = parseInt(document.getElementById("sailMax").value);
      
      fetch("/update_sail_config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ min: min, mid: mid, max: max })
      })
      .then(response => response.json())
      .then(data => {
        const slider = document.getElementById("sailSlider");
        slider.min = data.min;
        slider.max = data.max;
        slider.value = data.mid;
        document.getElementById("sailValue").innerText = data.mid;
      });
    }
    
    // Rudder slider event: send updated duty cycle
    const rudderSlider = document.getElementById("rudderSlider");
    rudderSlider.oninput = function() {
      document.getElementById("rudderValue").innerText = this.value;
      fetch("/set_rudder", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ duty: parseInt(this.value) })
      })
      .then(response => response.json())
      .then(data => console.log("Rudder duty set:", data));
    }
    
    // Sail slider event: send updated duty cycle
    const sailSlider = document.getElementById("sailSlider");
    sailSlider.oninput = function() {
      document.getElementById("sailValue").innerText = this.value;
      fetch("/set_sail", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ duty: parseInt(this.value) })
      })
      .then(response => response.json())
      .then(data => console.log("Sail duty set:", data));
    }
    
    // Fetch current windvane angle every 100 ms
    setInterval(() => {
      fetch("/windvane")
        .then(response => response.json())
        .then(data => {
          document.getElementById("windvaneAngle").innerText = data.angle.toFixed(2);
        });
    }, 100);
    
    // Calibrate windvane: set the current raw sensor reading as the new zero reference
    function calibrateWindvane() {
      fetch("/calibrate_windvane", {
        method: "POST",
        headers: { "Content-Type": "application/json" }
      })
      .then(response => response.json())
      .then(data => {
        document.getElementById("calibrateMsg").innerText = data.message;
      });
    }
  </script>
</body>
</html>
"""

app = Flask(__name__)

@app.route('/')
def index():
    return render_template_string(html_page, rudder_config=rudder_config, sail_config=sail_config)

@app.route('/update_rudder_config', methods=['POST'])
def update_rudder_config():
    global rudder_config
    data = request.get_json()
    try:
        rudder_config['min'] = int(data['min'])
        rudder_config['mid'] = int(data['mid'])
        rudder_config['max'] = int(data['max'])
    except Exception:
        return jsonify({"error": "Invalid configuration values"}), 400
    return jsonify(rudder_config)

@app.route('/update_sail_config', methods=['POST'])
def update_sail_config():
    global sail_config
    data = request.get_json()
    try:
        sail_config['min'] = int(data['min'])
        sail_config['mid'] = int(data['mid'])
        sail_config['max'] = int(data['max'])
    except Exception:
        return jsonify({"error": "Invalid configuration values"}), 400
    return jsonify(sail_config)

@app.route('/set_rudder', methods=['POST'])
def set_rudder():
    data = request.get_json()
    try:
        duty = int(data['duty'])
    except Exception:
        return jsonify({"error": "Invalid duty value"}), 400
    pi.set_servo_pulsewidth(RUDDER_PIN, duty)
    return jsonify({"message": "Rudder duty cycle set", "duty": duty})

@app.route('/set_sail', methods=['POST'])
def set_sail():
    data = request.get_json()
    try:
        duty = int(data['duty'])
    except Exception:
        return jsonify({"error": "Invalid duty value"}), 400
    pi.set_servo_pulsewidth(SAIL_PIN, duty)
    return jsonify({"message": "Sail duty cycle set", "duty": duty})

@app.route('/windvane', methods=['GET'])
def get_windvane():
    return jsonify({"angle": windvane_angle})

@app.route('/calibrate_windvane', methods=['POST'])
def calibrate_windvane():
    global windvane_offset, last_raw_value
    try:
        windvane_offset = -last_raw_value
        message = f"Calibration complete. New offset set to {windvane_offset}."
        print(message)
        return jsonify({"message": message})
    except Exception as e:
        return jsonify({"error": f"Calibration failed: {e}"}), 500

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)
    finally:
        # Clean up: reset PWM signals, stop pigpio, and close the I2C bus
        pi.set_servo_pulsewidth(RUDDER_PIN, 0)
        pi.set_servo_pulsewidth(SAIL_PIN, 0)
        pi.stop()
        bus.close()
