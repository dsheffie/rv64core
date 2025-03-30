#!/usr/bin/env python3

import os
import sys

def write_rtl(lg_bits):
    bits = 2**lg_bits
    o = open('ctz' + str(bits) + '.sv', 'w')
    o.write('module ctz%d(A, Y);\n' % bits)
    o.write('input logic [%d:0] A;\n' % (bits-1))
    o.write('output logic [%d:0] Y;\n' % (lg_bits))

    o.write('always_comb\n')
    o.write('begin\n')
    o.write('  Y = \'d0;\n')
    o.write('  casez(A)\n')
    for i in range(0, bits):
        j = bits - i
        s = ''
        for p in range(0, j):
            s = s + '0'
        if j != bits:
            s = s + '1'
            for p in range(j+1, bits):
                s = s + '?'
        s = ''.join(reversed(s))
        o.write("   %d'b%s: Y = 'd%d;\n" % (bits,s,j))
    
    o.write('  default:\n    begin\n    end\n')
    o.write('  endcase\n')
    o.write('end\n')

    
    o.write('endmodule\n')
    o.close()


if __name__ == '__main__':
    lg_bits = int(sys.argv[1])
    write_rtl(lg_bits)
