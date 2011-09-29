from __future__ import division

import math

import pygame
import serial

d = pygame.display.set_mode((640, 480))
s = serial.Serial("/dev/ttyACM0", 115200)

s.read(s.inWaiting())

last_guess = None

while True:
    line = s.readline()
    try:
        data = map(int, line.split(' '))
        assert len(data) == 8
        assert min(data) > 0 and max(data) <= 40000
    except:
        print "bad line", repr(line)
        continue
    
    data = [1/(x + 1) for x in data]
    print ' '.join('%.5f' % x for x in data)
    #print ' '.join(map(str, data))
    
    def conv_color(x):
        x = min(1, 500*x)
        return int(x * 255 + .5)
    d.blit(d, (0, -1))
    for x in xrange(640):
        c = conv_color(data[x * 8 // 640])
        d.set_at((x, 479), (c, c, c))
    
    guess = sum(i*v/sum(data) for i, v in enumerate(data))
    guess_stddev = math.sqrt(sum((i-guess)**2 * v/sum(data) for i, v in enumerate(data)))
    print guess_stddev
    
    def conv(x):
        return int((x + .5) * d.get_width() / 8)
    
    if last_guess is not None and guess_stddev < 2.2:
        pygame.draw.line(d, (255, 0, 0), (conv(last_guess - last_guess_stddev), 478), (conv(guess - guess_stddev), 479))
        pygame.draw.line(d, (0, 255, 0), (conv(last_guess), 478), (conv(guess), 479))
        pygame.draw.line(d, (255, 0, 0), (conv(last_guess + last_guess_stddev), 478), (conv(guess + guess_stddev), 479))
    last_guess = guess
    last_guess_stddev = guess_stddev

    pygame.display.update()

