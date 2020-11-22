# Script will stop neato lidar.
# If the ros neato driver fails the lidar continues to
# spin. This scrip is used to stop the spinning, especially
# during development.

# Author: Brannon Vann brannon.vann@gmail.com
# License: MIT

# Run this script: python stop_neato_lidar.py

import serial

try:
    serial = serial.Serial('/dev/ttyACM0', timeout=1)
    serial.write('setldsrotation off\n')
    serial.write('setled buttonoff\n')
    serial.write('testmode off\n')

    # close serial port
    serial.close()
    print("Done stopping down neato.")
except:
    print("an error occurred while stoping neato lidar.")
