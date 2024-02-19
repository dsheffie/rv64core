#!/usr/bin/python3

import subprocess
import glob
import re

if __name__ == '__main__':
    tests = glob.glob('*.rv64')
    passed = 0

    dsheffie = {}
    dsheffie_icnt = {}
    spike = {}
    
    min_icnt = 1000000000
    min_icnt_test = None

    with open('jobs.txt', 'w') as o:
        for test in tests:
            o.write('../rv32_core -f %s --maxicnt %d &> %s.out\n' % (test, 16*1024*1024, test)) 


