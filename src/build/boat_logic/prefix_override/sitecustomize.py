import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/boat/Desktop/TAFLAB_boatpi_roshumble/src/install/boat_logic'
