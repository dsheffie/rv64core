#!/usr/bin/python3
import numpy as np
import scipy.stats as st
import subprocess
import glob
import time
import re
import os
import argparse
import datetime

def run(ckpts, insns, lat):
    icnt = str(insns)
    memlat = str(lat)
    num_proc = os.cpu_count()
    children = []
    logs = []
    starttime = datetime.datetime.now()
    remaining_jobs = len(ckpts)
    for job in ckpts:
        log = job + '.txt'
        logs.append(log)
        with open(log, 'w') as out:
            args = ['/home/dsheffie/code/rv64core/rv64_core', '-f',\
                    job, '-w', '1',\
                    '-d', '1', '-c', '0',\
                    '--memlat', memlat, \
                    '--maxicnt', icnt]
            p = subprocess.Popen(args, stdout=out, stderr=out)
            children.append(p)
            
        while len(children) > (num_proc-1):
            completed = len(children)
            for c in children:
                children = [c for c in children if c.poll() is None]
            completed = completed - len(children)
            if(completed > 0):
                remaining_jobs = remaining_jobs - completed                
                print('remaining_jobs = %d' % remaining_jobs)
            time.sleep(1)
        
    
    while len(children) != 0:
        for c in children:
            children = [c for c in children if c.poll() is None]
        time.sleep(1)

    runtime = datetime.datetime.now() - starttime
    print('run took %s' % runtime)
    return logs

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--icnt", type=int, help="insns to sim", default=32*1024*1024)
    parser.add_argument("--lat", type=int, help="mem latency", default=30)
    parser.add_argument("--interval", type=int, help="simpoint interval", default=100*1000*1000)
    args = parser.parse_args()

    ckpts = glob.glob('*.rv64.chpt')
    logs = run(ckpts,args.icnt,args.lat)

    results = dict()
    deltas = []
    for log in logs:
        m = re.search(r'(\d+).rv64.chpt.txt$', log)
        i = int(m.groups()[0])        
        deltas.append(i)
        results[i] = log
        
    deltas.sort()
    m = len(deltas)//2

    checkpoint_id_map = dict()
    icnt_to_weight_map = dict()
    with open('simpoints', 'r') as in_:
        for line in in_:
            m = re.search(r'(\d+)\s(\d+)', line)
            g = m.groups()
            icnt = int(g[0])*args.interval
            checkpoint_id_map[int(g[1])] = icnt
            
    with open('weights', 'r') as in_:
        for line in in_:
            m = re.search(r'(\d+.\d+)\s(\d+)', line)
            g = m.groups()
            i = int(g[1])
            w = float(g[0])
            icnt = checkpoint_id_map[i]
            icnt_to_weight_map[icnt] = w
    
    perf = {}
    total_cycles = 0
    total_insns = 0
    a = []
    weighted_cpi = 0.0    
    for log in logs:
        m = re.search(r'chpt(\d+).rv64', log)
        icnt = int(m.groups()[0])
        weight = icnt_to_weight_map[icnt]
        with open(log, 'r') as in_:
            icnt = None
            cycles = None
            for line in in_:
                if (icnt != None) and (cycles != None):
                    break
                
                m = re.search(r'total_retire\s+=\s+(\d+)', line)
                if m:
                    icnt = m.groups()[0]
                    continue
                m = re.search(r'total_cycle\s+=\s+(\d+)', line)
                if m:
                    cycles = m.groups()[0]
                    continue
            cpi = float(cycles)/float(icnt)
            
            perf[log] = cpi
            a.append(cpi)
            weighted_cpi += weight * cpi

    print('predicted ipc = %g' % (1.0/weighted_cpi))
