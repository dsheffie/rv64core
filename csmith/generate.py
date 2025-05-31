#!/usr/bin/python3

import subprocess
import re
import time

if __name__ == '__main__':
    n_tests = 10
    use_paging = False
    num_proc = 24
    children = []
    
    for t in range(0,n_tests):
        r = 'test-' + str(t)
        test = r + '.c'
        hitTimeOut = True
        with open(test, 'w') as o:
            p = subprocess.Popen(['csmith','--no-argc'], stdout = o, stderr = o)
            children.append(p)

        while len(children) > (num_proc-1):
            children_ = []
            for c in children:
                if c.poll() is None:
                    children_.append(c)
            children = children_

    
    while len(children) != 0:
        children_ = []
        for c in children:
            if c.poll() is None:
                children_.append(c)
        children = children_

    print('done generating')

    for t in range(0,n_tests):
        r = 'test-' + str(t)
        test = r + '.c'
        hitTimeOut = True
        with open('/dev/null', 'w') as o:
            p = subprocess.Popen(['/opt/riscv64/bin/riscv64-unknown-elf-gcc', '-O3', '-march=rv64ima_zicsr', '-mcmodel=medany', \
                            '-I/usr/include/csmith/', test, '-o', r+'.rv64', '-specs=htif.specs'], stderr=o, stdout=o)
            children.append(p)

        with open('/dev/null', 'w') as o:
            p = subprocess.Popen(['/opt/riscv64/bin/riscv64-unknown-elf-gcc', '-O3', '-march=rv64ima_zicsr_zbb', '-mcmodel=medany', \
                            '-I/usr/include/csmith/', test, '-o', r+'.zbb.rv64', '-specs=htif.specs'], stderr=o, stdout=o)
            children.append(p)
            
        while len(children) > (num_proc-1):
            children_ = []
            for c in children:
                if c.poll() is None:
                    children_.append(c)
            children = children_

                
    while len(children) != 0:
        children_ = []
        for c in children:
            if c.poll() is None:
                children_.append(c)
        children = children_

            
            
