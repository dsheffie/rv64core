#!/usr/bin/python3

import glob
import os
import subprocess

def main():
    svs = glob.glob('*.sv')
    if not os.path.isdir('verilog'):
        os.mkdir('verilog')
        
    for sv in svs:
        r = sv.split('.sv')[0]
        v = r+'.v'
        cmd = ['sv2v', sv, '--write=verilog/'+v]
        subprocess.run(cmd)

if __name__ == "__main__":
    main()
