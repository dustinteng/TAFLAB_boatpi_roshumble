#!/usr/bin/env python3
import time
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver

def on_update(dev):
    # dev here is your DeviceModel instance in the callback
    raw = dev.getDeviceData("accX")
    try:
        val = float(raw)
    except (TypeError, ValueError):
        val = None
    print(f"accX raw: {raw!r}  →  float: {val}")

if __name__ == "__main__":
    # 1) Instantiate device just like in your node
    device = deviceModel.DeviceModel(
        "myJY901",
        WitProtocolResolver(),
        JY901SDataProcessor(),
        "51_0"
    )

    # 2) Configure your serial port & baud
    device.serialConfig.portName = "/dev/ttyUSB0"   # adjust if yours is different
    device.serialConfig.baud     = 9600

    # 3) Open the device and attach callback
    device.openDevice()
    device.dataProcessor.onVarChanged.append(on_update)

    print("Listening for accX…  (Ctrl‑C to quit)\n")
    try:
        # keep the script alive so callbacks keep firing
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nExiting.")
        # if your API has a close/shutdown method, call it here
