#!/usr/bin/python3

import subprocess
import re
import glob
import pathlib

if __name__ == '__main__':
    failures = []
    successes = []
    badvas = []
    
    timeout = []
    zero_disp_loops = 0
    for test in glob.glob('*.out'):
        failed = False
        succeed = False
        badva = False
        icnt = None
        zero_disp_loop = False
        with open(test, 'r') as in_:
            for line in in_:
                m = re.search(r'checksum\s=\s[0-9A-F]+', line)
                if m != None:
                    succeed = True
                    successes.append(test)
                    break
                m = re.search(r'register [a-z0-9]+ does not match', line)
                if m != None:
                    failed = True
                m = re.search(r'GOT VA', line)
                if m != None:
                    badva = True
                m = re.search(r'instructions retired = (\d+)', line)
                if m:
                    g = m.groups()
                    icnt = int(g[0])
                m = re.search(r'j 0,', line)
                if m:
                    zero_disp_loop = True

        if failed:
            failures.append((icnt, test))

        if badva:
            badvas.append(test)
            t = test.split('.')
            pathlib.Path.unlink(test)
            pathlib.Path.unlink(t[0] +'.' + t[1])
            
        if not(succeed) and not(failed) and not(badva):
            timeout.append(test)
            if zero_disp_loop:
                zero_disp_loops = zero_disp_loops + 1
            t = test.split('.')
            pathlib.Path.unlink(test)
            pathlib.Path.unlink(t[0] +'.' + t[1])
                
    print('%d zero displacement loops' % zero_disp_loops)
    failures.sort()
    for f in failures:
        print('%d,%s' % f)

    print('%d successes, %d failures, %d badva %d, timeout' % (len(successes), len(failures), len(badvas), len(timeout)))

    with open('timeout.txt', 'w') as o:
        for t in timeout:
            o.write('%s\n' % t)

    with open('tjobs.txt', 'w') as o:
        for test in timeout:
            t = test.split('.')
            job =t[0] +'.' + t[1]
            o.write('./rv64_core -f %s --maxicnt %d &> %s\n' % (job, 32*1024*1024, test)) 
