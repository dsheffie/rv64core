#!/usr/bin/python3

import subprocess
import glob
import re

if __name__ == '__main__':
    tests = glob.glob('*.rv32')
    passed = 0

    dsheffie = {}
    dsheffie_icnt = {}
    spike = {}
    
    min_icnt = 1000000000
    min_icnt_test = None
    
    for test in tests:
        with open(test + '.rv32.out', 'w') as o:
            try:
                subprocess.run(['../ooo_core', '-f', test], stdout=o, stderr=o, timeout=15)
            except subprocess.TimeoutExpired:
                print('failure : timeout for %s' % test)
                continue

        got_checksum = False
        with open(test + '.rv32.out', 'r') as i:
            for line in i:
                m = re.search('checksum', line)
                if m:
                    got_checksum = True
                    break
        if got_checksum == False:
            print('test %s didnt generate a checksum' % test)
