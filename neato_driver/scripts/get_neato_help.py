# Script used to read all help text from Neato.
# Simply connect Neato and run this script. 
# All helptext is written to a file in the 
# same directory called neato_help.txt

# Author: Brannon Vann
# License: MIT

import serial
import threading

serial = serial.Serial('/dev/ttyACM0', timeout=1)
print(serial.name)
serial.write('help\n')
serial.flush()
helpText = "Help output from Neato"

h = serial.readlines()

def printLines(line):
    global helpText 
    helpText += line
    print(line),

def helpResponseProcessor(line):
   if line.find(' - ') != -1:
     parts = line.split(" - ")
     serial.write("help " + parts[0] + '\n')
     serial.flush()
     incoming = serial.readlines()
     map(printLines, incoming)

def GetData(line):
   if line.find(' - ') != -1 and line.startswith('Get'):
     parts = line.split(" - ")
     serial.write(parts[0] + '\n')
     serial.flush()
     incoming = serial.readlines()
     map(printLines, incoming)

# print help output
map(printLines, h)
# iterate help output to request command specific output
map(helpResponseProcessor, h)

# get all data from neato
map(GetData, h)

#close serial port
serial.close()

#write out file, overwrites any existing file
helpResponseProcessor = open("neato_help.txt", "w")
helpResponseProcessor.write(helpText)
helpResponseProcessor.close()
