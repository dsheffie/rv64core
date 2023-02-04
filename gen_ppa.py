#!/usr/bin/env python3

import os
import sys

def write_rtl(lg_bits):
    bits = 2**lg_bits
    o = open('ppa' + str(bits) + '.v', 'w')
    o.write('module ppa%d(A, B, Y);\n' % bits)
    o.write('input [%d:0] A;\n' % (bits-1))
    o.write('input [%d:0] B;\n' % (bits-1))
    o.write('output [%d:0] Y;\n' % (bits-1))

    #generate the first level of propagate and generates
    gg = []
    pp = []
    for i in range(0, bits):
        n = 'pp_'+str(i)
        g = 'gg_'+str(i)
        pp.append(n)
        gg.append(g)
        o.write('wire %s = A[%d] ^ B[%d];\n' % (n,i,i))
        o.write('wire %s = A[%d] & B[%d];\n' % (g,i,i))
        
    # save initial prop bits
    p = pp
        
    #generate parallel-prefix network of prop
    for l in range(0, lg_bits):
        d = 2**l
        tpp = []
        tgg = []
        for i in range(0, bits):
            n = pp[i]
            g = gg[i]
            if (i-d) > -1:
                n = n + '_' + pp[i-d]
                g = g + '_' + gg[i-d]
                o.write('wire %s = %s & %s;\n' % (n, pp[i], pp[i-d]))
                o.write('wire %s = (%s & %s) | %s;\n' % (g, gg[i-d], pp[i], gg[i]))
                   
            tpp.append(n)
            tgg.append(g)       
            
        pp = tpp
        gg = tgg
        
    o.write('assign Y[0] = %s;\n' % p[0])

    
    for i in range(1, bits):
        o.write('assign Y[%d] = %s ^ %s;\n' % (i, p[i], gg[i-1] ))
    

    o.write('endmodule\n')
    o.close()


if __name__ == '__main__':
    lg_bits = int(sys.argv[1])
    write_rtl(lg_bits)
