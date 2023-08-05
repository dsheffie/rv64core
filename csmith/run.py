#!/usr/bin/python3

import subprocess
import glob
import re
import os

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
                subprocess.run(['../rv32_core', '-f', test], stdout=o, stderr=o, timeout=15)
            except subprocess.TimeoutExpired:
                print('failure : timeout for %s' % test)
                for g in glob.glob(test+'*'):
                    os.remove(g)
                continue

        got_checksum = False
        got_VA = False
        with open(test + '.rv32.out', 'r') as i:
            for line in i:
                m = re.search('checksum', line)
                if m:
                    got_checksum = True
                    break
                m = re.search('GOT VA for', line)
                if m:
                    got_VA = True
                    break

        if got_checksum == True:
            continue
        elif got_VA == True:
            print('test %s generated bad address' % test)
            for g in glob.glob(test+'*'):
                os.remove(g)
        else:
            print('test failed!' % test)
