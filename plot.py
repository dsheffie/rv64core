#!/usr/bin/python3

import re
import sys
import matplotlib.pyplot as plt
import numpy

if __name__ == '__main__':
    cycles = []
    ipc = []
    with open(sys.argv[1], 'r') as in_:
        for line in in_:
            m = re.search('cycle\s(\d+)\s[0-9a-f]+(.*),\s(\d+.\d+)', line)
            if m == None:
                continue

            c = int(m.groups()[0])
            t = float(m.groups()[2])
            
            cycles.append(c)
            ipc.append(t)

#    f = numpy.abs(numpy.fft.fft(ipc))
#    fig, ax = plt.subplots()
#    ax.plot(f)
#    ax.grid()
#    r = sys.argv[1].split('.')[0]
#    fig.savefig(r + ".pdf")
#    plt.show()
            
    fig, ax = plt.subplots()
    ax.plot(cycles,ipc)
    ax.grid()
    r = sys.argv[1].split('.')[0]
    fig.savefig(r + ".pdf")
    plt.show()

    
