#!/usr/bin/python

import argparse
import subprocess
import sys
import time
import traceback

import serial

parser = argparse.ArgumentParser()

parser = argparse.ArgumentParser(description='Program a microcontroller running the chip45boot2 bootloader')
parser.add_argument('-p', '--port',
    help='open port at specified path (default: /dev/ttyACM0)',
    type=str, action='store', default='/dev/ttyACM0', dest='port')
parser.add_argument('-b', '--baudrate',
    help='communicate at specified baud rate (default: 115200)',
    type=int, action='store', default=115200, dest='baudrate')
parser.add_argument('-e', '--eeprom',
    help='program the EEPROM instead of the flash memory',
    action='store_true', default=False, dest='eeprom')
parser.add_argument(metavar='HEX_FILE',
    help='path to Intel HEX file to read',
    type=str, action='store', dest='hex_file')
args = parser.parse_args()

hex_file_lines = subprocess.check_output(["srec_cat", args.hex_file, "-Intel", "-Output", "-", "-Intel", "-Line_Length", "44"]).splitlines(True)
s = serial.Serial(args.port, args.baudrate)

class UnexpectedError(Exception):
    pass

def expect(*choices):
    pos = 0
    res = ''
    while True:
        if len(choices) == 1 and pos == len(choices[0]):
            return choices[0]
        if len(choices) == 0:
            raise UnexpectedError(repr(res))
        c = s.read(1)
        res += c
        choices = [x for x in choices if x[pos] == c]
        pos += 1

while True:
    try:
        print 'Attempting communication...'
        
        while True:
            data = ''
            s.setDTR(True)
            time.sleep(.1)
            s.setDTR(False)
            for i in xrange(5):
                time.sleep(.025)
                s.write('U'*10)
                time.sleep(.025)
                data += s.read(s.inWaiting())
                if data.endswith('c45b2 v2.9C\n\r>\x11'):
                    break
            else:
                continue
            break
        
        print 'Communication established!'
        
        # test communication
        s.write('\n')
        expect('\x13-\n\r>\x11')
        
        # start programming
        if args.eeprom:
            s.write('pe\n')
            expect('\x13pe+\n\r\x11')
        else:
            s.write('pf\n')
            expect('\x13pf+\n\r\x11')
        
        # send data, displaying progress
        for i, line in enumerate(hex_file_lines):
            s.write(line)
            expect('\x13.')
            if i == len(hex_file_lines) - 1:
                e = '+\n\r>\x11'
            else:
                e = '\x11'
            sys.stdout.write('.')
            if expect(e, '*' + e).startswith('*'):
                sys.stdout.write('*')
            sys.stdout.flush()
        sys.stdout.write('\n')
        
        # start program
        s.write('g\n')
        expect('\x13g+\n\r\x11')
        
        print 'Success!'
    except UnexpectedError:
        traceback.print_exc()
        print 'Communications error, retrying...'
        continue
    break
