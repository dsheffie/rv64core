#!/usr/bin/python3

import subprocess
import re
import glob

if __name__ == '__main__':
    failures = []
    successes = []
    timeout = []
    for test in glob.glob('*.out'):
        failed = False
        succeed = False
        icnt = None
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
                m = re.search(r'instructions retired = (\d+)', line)
                if m:
                    g = m.groups()
                    icnt = int(g[0])
                

        if failed:
            failures.append((icnt, test))

        if not(succeed) and not(failed):
            timeout.append(test)

    failures.sort()
    for f in failures:
        print('%d,%s' % f)

    print('%d successes, %d failures, %d timeout' % (len(successes), len(failures), len(timeout)))

    #for t in timeout:
    #    print(t)
