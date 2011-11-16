from __future__ import division

import math
import colorsys

import pygame
import serial

d = pygame.display.set_mode((1100, 700))
s = serial.Serial("/dev/ttyACM0", 115200)

s.read(s.inWaiting())
s.write('mf')

old_data = None

while True:
    line = s.readline()
    try:
        data = filter(None, line.split(' '))
        assert data[0] == 'MC'
        data = map(float, data[1:5])
        #assert len(data) == 3
    except:
        print "bad line", repr(line)
        if 'Starting up' in line:
            s.write('mr')			# Changed from mf to mr temporarily
        continue
    
    print data
    d.blit(d, (-1, 0))
    pygame.draw.line(d, (0, 0, 0), (d.get_width()-1, 0), (d.get_width()-1, d.get_height()-1))
    
    def conv(x):
        return d.get_height() - int((x/4) * d.get_height())
    
    if old_data is not None:
        for i in xrange(len(data)):
            pygame.draw.line(d, colorsys.hsv_to_rgb(i/len(data), 1, 255), (d.get_width()-2, conv(old_data[i])), (d.get_width()-1, conv(data[i])))
    old_data = data

    pygame.display.update()

