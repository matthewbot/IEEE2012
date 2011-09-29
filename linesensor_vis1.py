from __future__ import division

import pygame
import serial

s = serial.Serial('/dev/ttyACM0', 115200)
d = pygame.display.set_mode((640, 480))

while True:
   line = s.readline()
   try:
       data = map(float, line.split(' '))
       assert len(data) == 9
   except:
       print "bad line:", repr(line)
       continue
   
   data2 = [min(1, 500*1/(1+x)) for x in data[:-1]]
   pos = (data[-1]+1)/2

   d.blit(d, (0, -1))
   for x in xrange(d.get_width()):
        g = int(255 * data2[x * 8 // d.get_width()])
        d.set_at((x, d.get_height() - 1), (g, g, g))
   d.set_at((int(pos*d.get_width()), d.get_height() - 1), (0, 255, 0))
   pygame.display.update()
   
