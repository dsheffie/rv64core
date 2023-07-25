#!/usr/bin/python3

import subprocess
import re

if __name__ == '__main__':
    n_tests = 10
    for t in range(0,n_tests):
        r = 'test-' + str(t)
        test = r + '.c'
        hitTimeOut = True
        print('generating test %d' % t)
        with open(test, 'w') as o:
            subprocess.run(['csmith'], stdout = o, stderr = o)
        with open('/dev/null', 'w') as o:
            subprocess.run(['riscv32-unknown-elf-gcc', '-mabi=ilp32', '-march=rv32i', '-O3', '-I/usr/include/csmith/', test, '-o', r+'.rv32', '-specs=htif.specs'], stderr=o, stdout=o)

            # with open('/dev/null', 'w') as o:
            #     try:
            #         subprocess.run(['interp_mips', '-f', r+'.mips'], timeout=60,stdout=o, stderr=o)
            #     except subprocess.TimeoutExpired:
            #         print('timeout!')
            #         #hitTimeOut =  True
            #         #continue

            # print('hitTimeOut = %d' % hitTimeOut)
