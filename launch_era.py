#!/usr/bin/env python

import subprocess
import shlex
import os, signal
import sys

n_cars  = 2
bagfile = '/dccstor/epochs/aporvaa/hetero_era/data/2020-09-10-14-43-09.bag'
src_dir = '/dccstor/epochs/aporvaa/hetero_era/XF_x86_hpvm'
bin_dir = '/dccstor/epochs/aporvaa/hetero_era/XF_x86_hpvm'

processes = []
cwd = os.getcwd()


def close_port(port):
    
    command = "lsof -i :%s | awk '{print $2}'" % port
    pids = subprocess.check_output(command, shell=True)
    pids = pids.strip().decode("utf-8")
    for pid in pids.split('\n'):
        if pid.isnumeric():
            print('[' + pid + ']')
            os.kill(int(pid), signal.SIGKILL)


def main():

    n_steps = ''
    if len(sys.argv) > 2:
        print('Usage: ' + sys.argv[0] + ' [n_steps > 0]')
        sys.exit(-1)
    elif len(sys.argv) == 2:
        if int(sys.argv[1])<1:
            print('Usage: ' + sys.argv[0] + ' [n_steps > 0]')
            sys.exit(-1)
        steps = sys.argv[1]
        n_steps = '-s ' + str(steps)       
        print('***** Starting ERA with ' + str(n_cars) + ' cars (max steps: ' + str(steps) + ') *****\n')
    else:
        print('***** Starting ERA with ' + str(n_cars) + ' cars *****\n')
    
    print('>>> Closing open ports from previous runs')
    close_port(5556)    # Bag server (car 1)
    close_port(5557)    # Bag server (car 2)
    close_port(5558)    # XMIT server (car 1)
    close_port(5559)    # RECV server (car 1)
    close_port(5560)    # XMIT server (car 2)
    close_port(5561)    # RECV server (car 2)
    close_port(5562)    # Car server (car 1)
    close_port(5563)    # Car server (car 2)
    print()
    
    print('>>> Starting bag (trace) reading')
    for car in range(n_cars):
        print('  - For car ' + str(car))
        cmd = 'python %s/read_bag_%d.py %s' % (src_dir, car+1, bagfile)
        print('    Command: ' + cmd)
        with open(cwd + '/read_bag_' + str(car+1) + '.out','w') as out:
            p = subprocess.Popen(shlex.split(cmd), stdout=out, stderr=subprocess.STDOUT, cwd=src_dir)
            processes.append(p)
    print()

    print('>>> Starting V2V transceiver')
    for car in range(n_cars):
        print('  - For car ' + str(car))
        cmd = '%s/wifi_comm_%d.sh' % (src_dir, car+1)
        print('    Command: ' + cmd)
        with open(cwd + '/wifi_comm_' + str(car+1) + '.out','w') as out:
            p = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, cwd=src_dir)
            processes.append(p)
    print()

    print('>>> Starting CARLA interface')
    for car in range(n_cars):
        print('  - For car ' + str(car))
        cmd = '%s/carla_recvr_%d.sh' % (src_dir, car+1)
        print('    Command: ' + cmd)
        with open(cwd + '/carla_recvr_' + str(car+1) + '.out','w') as out:
            p = subprocess.Popen(shlex.split(cmd), stdout=out, stderr=subprocess.STDOUT, cwd=src_dir)
            processes.append(p)
    print()

    print('>>> Starting ERA engines')
    for car in range(n_cars):
        print('  - For car ' + str(car))
        cmd = '%s/era%d %s' % (bin_dir, car+1, n_steps)
        print('    Command: ' + cmd)
        with open(cwd + '/era_' + str(car+1) + '.out','w') as out:
            p = subprocess.Popen(shlex.split(cmd), stdout=out, stderr=subprocess.STDOUT, cwd=bin_dir)
            processes.append(p)
    print()
    
    print('>>> Waiting for all processes to complete... ', end='', flush=True)
    exit_codes = [p.wait() for p in processes]
    print('DONE!')



if __name__ == '__main__':
   main()


