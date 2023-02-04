#!/usr/bin/env python3

import os
import sys

def write_rtl(bits):
    with open('rca' + str(bits) + '.v', 'w') as o:
        o.write('module rca%d(A, B, Y);\n' % bits)
        o.write('input [%d:0] A;\n' % (bits-1))
        o.write('input [%d:0] B;\n' % (bits-1))
        o.write('output [%d:0] Y;\n' % (bits-1))


        o.write('assign Y[0] = A[0] ^ B[0];\n')
        o.write('wire wC0 = A[0] & B[0];\n')

        for i in range(1, bits):
            o.write('assign Y[%d] = A[%d] ^ B[%d] ^ wC%d;\n' % (i, i, i, i-1))
            if i != (bits-1):
                o.write('wire wC%d = (A[%d] & B[%d]) | (wC%d & (A[%d] ^ B[%d]));\n' % (i, i, i, i-1, i, i))
    

        o.write('endmodule\n')


if __name__ == '__main__':
    bits = int(sys.argv[1])
    write_rtl(bits)
