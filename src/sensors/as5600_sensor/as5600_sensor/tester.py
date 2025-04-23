# config_loader_single.py

import os
import json

class ConfigLoader:
    def __init__(self, filename="/home/boat/Desktop/version4/TAFLAB_boatpi_roshumble/src/config.json"):
        self.filename = filename
        self.config_data = self.load_config()

    def load_config(self):
        """Load config data from config.json."""
        config_path = os.path.join(os.path.dirname(__file__), self.filename)
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                try:
                    return json.load(f)
                except json.JSONDecodeError:
                    print("Error: JSON file could not be decoded.")
                    return {}
        else:
            print(f"Config file not found at {config_path}")
            return {}

    def get(self, key, default=None):
        """Get a configuration value with an optional default."""
        return self.config_data.get(key, default)

# Example usage
if __name__ == "__main__":
    config = ConfigLoader()
    servo_min = config.get("windvane_offset", 1000)
    servo_max = config.get("windvane_reverse", False)
    print(f"Servo Min: {servo_min}, Servo Max: {servo_max}")
