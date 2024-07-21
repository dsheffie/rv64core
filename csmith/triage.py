#!/usr/bin/python3

import subprocess
import re
import glob

if __name__ == '__main__':
    failures = []
    for test in glob.glob('*.out'):
        failed = False
        icnt = None
        with open(test, 'r') as in_:
            for line in in_:
                m = re.search(r'register [a-z0-9]+ does not match', line)
                if m != None:
                    failed = True
                m = re.search(r'instructions retired = (\d+)', line)
                if m:
                    g = m.groups()
                    icnt = int(g[0])
                

        if failed:
            failures.append((icnt, test))


    failures.sort()
    for f in failures:
        print('%d,%s' % f)
