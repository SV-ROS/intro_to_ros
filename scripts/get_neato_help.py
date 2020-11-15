# Script used to read all help text from Neato.
# Simply connect Neato and run this script.
# All markup is written to a file in the
# same directory called neato_help.txt

# Author: Brannon Vann brannon.vann@gmail.com
# License: MIT

# Run this script: python get_neato_help.py

# Note: This script does not save your serial numbers. #Prevent Serial Write parts below prevent the write out serial numbers.

import serial
import threading

serial = serial.Serial('/dev/ttyACM0', timeout=1)
print(serial.name)
serial.write('help\n')
serial.flush()
markup = "# Neato Help\n"
markup += '\n'
markup += "   This document contains help documentation acquired from neato by running the script titled get_neato_help.py contained in the project at <https://github.com/brannonvann/neato>.\n"
markup += '\n'
markup += "## Help Command Output from Neato"
markup += '\n'

help = serial.readlines()


def printLines(lines):
    global markup
    i = 0
    while i < len(lines):
        line = lines[i]

        # first line
        if i == 0:
            if line.startswith('help'):
                markup += "\n## Command " + line + '\n'

            elif line.startswith('Get'):
                markup += "\n## Command " + line + '\n'
            else:
                markup += "Unidentified Line: " + line

        # other lines
        else:
            if line.find('Serial') != -1:
                # Prevent Serial Write
                markup = markup  # do nothing
                # markup += line # write serial out
            elif line.startswith('    '):
                markup += line
            elif line == '\n':
                markup += line
            elif line == ' ':
                markup += line
            else:
                markup += "    " + line

        print(line),
        i += 1


def helpResponseProcessor(line):
    if line.find(' - ') != -1:
        parts = line.split(" - ")
        serial.write("help " + parts[0] + '\n')
        # serial.flush()
        incoming = serial.readlines()
        printLines(incoming)


def GetData(line):
    if line.find(' - ') != -1 and line.startswith('Get'):
        parts = line.split(" - ")
        serial.write(parts[0] + '\n')
        # serial.flush()
        incoming = serial.readlines()
        printLines(incoming)


# print help output
printLines(help)
# iterate help output to request command specific output
map(helpResponseProcessor, help)

# get all data from neato
map(GetData, help)

# close serial port
serial.close()

# write out file, overwrites any existing file
helpResponseProcessor = open("neato_help.md", "w")
helpResponseProcessor.write(markup)
helpResponseProcessor.close()

print("Done creating neato_help.md document")
