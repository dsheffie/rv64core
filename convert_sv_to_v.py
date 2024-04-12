#!/usr/bin/python3

import glob
import os
import subprocess

def main():
    svs = glob.glob('*.sv')
    if not os.path.isdir('verilog'):
        os.mkdir('verilog')

    outputs = []
    for sv in svs:
        r = sv.split('.sv')[0]
        v = r+'.v'
        cmd = ['sv2v', sv, '--write=verilog/'+v, '-D=FPGA64_32']
        outputs.append('verilog/' + v)
        subprocess.run(cmd)

    with open('rv32core.v', 'w') as o:
        for output in outputs:
            with open(output, 'r') as in_:
                for line in in_:
                    o.write('%s' % line)
                o.write('\n\n')

if __name__ == "__main__":
    main()
