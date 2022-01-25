#!/usr/bin/env python

import subprocess
import shlex
import os

n_cars  = 2
bagfile = '/dccstor/epochs/ajvega/era/data/2020-09-10-14-43-09.bag'
src_dir = '/dccstor/epochs/ajvega/era/src'
bin_dir = '/dccstor/epochs/ajvega/era/build'

processes = []
cwd = os.getcwd()

def main():
    
    print("***** Starting ERA with " + str(n_cars) + " cars *****\n")
    
    print(">>> Starting bag (trace) reading")
    for car in range(n_cars):
        print("  - For car " + str(car))
        cmd = 'python %s/read_bag_%d.py %s' % (src_dir, car+1, bagfile)
        print("    Command: " + cmd)
        with open(cwd + '/read_bag_' + str(car+1) + '.out','w') as out:
            p = subprocess.Popen(shlex.split(cmd), stdout=out, stderr=subprocess.STDOUT, cwd=src_dir)
            processes.append(p)
    print()

    print(">>> Starting V2V transceiver")
    for car in range(n_cars):
        print("  - For car " + str(car))
        cmd = '%s/wifi_comm_%d.sh' % (src_dir, car+1)
        print("    Command: " + cmd)
        with open(cwd + '/wifi_comm_' + str(car+1) + '.out','w') as out:
            p = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, cwd=src_dir)
            processes.append(p)
    print()

    print(">>> Starting CARLA interface")
    for car in range(n_cars):
        print("  - For car " + str(car))
        cmd = '%s/carla_recvr_%d.sh' % (src_dir, car+1)
        print("    Command: " + cmd)
        with open(cwd + '/carla_recvr_' + str(car+1) + '.out','w') as out:
            p = subprocess.Popen(shlex.split(cmd), stdout=out, stderr=subprocess.STDOUT, cwd=src_dir)
            processes.append(p)
    print()

    print(">>> Starting ERA engines")
    for car in range(n_cars):
        print("  - For car " + str(car))
        cmd = '%s/era%d' % (bin_dir, car+1)
        print("    Command: " + cmd)
        with open(cwd + '/era_' + str(car+1) + '.out','w') as out:
            p = subprocess.Popen(shlex.split(cmd), stdout=out, stderr=subprocess.STDOUT, cwd=bin_dir)
            processes.append(p)
    print()
    
    print(">>> Waiting for all processes to complete... ", end='', flush=True)
    exit_codes = [p.wait() for p in processes]
    print('DONE!')



if __name__ == "__main__":
   main()


