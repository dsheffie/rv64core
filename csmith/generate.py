#!/usr/bin/python3

import subprocess
import re

if __name__ == '__main__':
    n_tests = 1000
    use_paging = False
    for t in range(0,n_tests):
        r = 'test-' + str(t)
        test = r + '.c'
        hitTimeOut = True
        print('generating test %d' % t)
        with open(test, 'w') as o:
            subprocess.run(['csmith','--no-argc'], stdout = o, stderr = o)

        src = []
        print(test)
        if use_paging:
            with open(test, 'r') as i:
                for line in i:
                    m = re.search(r'int main', line)
                    if m:
                        src.append('int main_()\n')
                    else:
                        src.append(line)
                    
            with open(test, 'w') as o:
                o.write('#include \"setup_paging.h\"\n');
                for l in src:
                    o.write('%s'% l)
        
            
        with open('/dev/null', 'w') as o:
            subprocess.run(['/opt/riscv64/bin/riscv64-unknown-elf-gcc', '-O3', '-march=rv64ima_zicsr', '-mcmodel=medany', \
                            '-I/usr/include/csmith/', test, '-o', r+'.rv64', '-specs=htif.specs'], stderr=o, stdout=o)

        with open('/dev/null', 'w') as o:
            subprocess.run(['/opt/riscv64/bin/riscv64-unknown-elf-gcc', '-O3', '-march=rv64ima_zicsr_zba_zbb', '-mcmodel=medany', \
                            '-I/usr/include/csmith/', test, '-o', r+'.zba.rv64', '-specs=htif.specs'], stderr=o, stdout=o)

            
            # with open('/dev/null', 'w') as o:
            #     try:
            #         subprocess.run(['interp_mips', '-f', r+'.mips'], timeout=60,stdout=o, stderr=o)
            #     except subprocess.TimeoutExpired:
            #         print('timeout!')
            #         #hitTimeOut =  True
            #         #continue

            # print('hitTimeOut = %d' % hitTimeOut)
