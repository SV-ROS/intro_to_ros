# Script will turn off neato.

# Author: Brannon Vann - brannon.vann@gmail.com
# License: MIT

# Run this script: python stop_neato_lidar.py

import serial

try:
    serial = serial.Serial('/dev/ttyACM0', timeout=1)
    serial.write('SetSystemMode Shutdown\n')
    # close serial port
    serial.close()
    print("Done shutting Down Neato")
except:
    print("An error occurred while shuting down neato.")
