#!/usr/bin/env python3
#coding: utf-8

import client
import time
import task
import tcp
import options
import sys
import os
import signal
import gamecontroller
import teammate
import subprocess
from options import OPTS
from config import CONF
from robot import ROBOT
from common import LOG

is_alive = False

def signal_handler(sig, frame):
    global is_alive
    if sig == signal.SIGINT:
        is_alive = False


def get_args():
    args = sys.argv[1:]
    args1 = []
    args2 = ''
    for arg in args:
        if '-p' in arg:
            args1.append(arg)
            args2 += ' %s'%arg
        elif '-h' in arg:
            args1.append(arg)
        elif '-t' in arg:
            args1.append(arg)
        elif '-g' in arg:
            args1.append(arg)
        else:
            args2 += ' %s'%arg
    return (args1, args2)


if __name__ == '__main__':
    signal.signal(signal.SIGINT,signal_handler)
    prog = './controller'
    (args1, args2) = get_args()
    if '-h' in args1:
        os.system('%s -h'%prog)
    OPTS.init(args1)
    CONF.init(OPTS.id)
    cmd = prog + args2
    controller = subprocess.Popen(cmd, shell=True)
    time.sleep(1)
    cl = client.client(int(CONF.get_config('net.tcp.port')))
    cl.attach(ROBOT)
    cl.start()
    is_alive = True
    while not cl.connected and is_alive:
        time.sleep(0.5)
    if not cl.connected:
        cl.dettach(ROBOT)
        cl.stop()
        exit(0)
    
    if OPTS.use_gamectrl:
        gc = gamecontroller.GameController(int(CONF.get_config('net.udp.gamectrl.recv_port')),int(CONF.get_config('net.udp.gamectrl.send_port')),\
            int(CONF.get_config('team_number')), OPTS.id, int(CONF.get_config('net.udp.gamectrl.period')))
        gc.attach(ROBOT)
        gc.start()
    if OPTS.use_teammate:
        tm = teammate.Teammate(int(CONF.get_config('net.udp.teammate.port')), int(CONF.get_config('net.udp.teammate.period')))
        tm.attach(ROBOT)
        tm.start()
    cl.regsit(tcp.tcp_cmd_type.TASK_DATA, tcp.tcp_data_dir.DIR_SUPPLY)
    cl.regsit(tcp.tcp_cmd_type.WM_DATA, tcp.tcp_data_dir.DIR_APPLY)
    t1 = task.task(task.task_type.TASK_WALK, (0.04, 0.02, 20.0, True))
    data = t1.data()
    t2 = task.task(task.task_type.TASK_ACT, ('reset',))
    data = t2.data()
    while is_alive:
        if cl.connected:
            cl.send(data)
        else:
            break
        try:
            #print('state: %d'%ROBOT.gc_data().state)
            #print('ballx: %f'%ROBOT.wm_data().ballx)
            pass
        except:
            pass
        time.sleep(1)

    controller.send_signal(signal.SIGINT)
    cl.dettach(ROBOT)
    cl.stop()
    if OPTS.use_teammate:
        tm.dettach(ROBOT)
        tm.stop()
    if OPTS.use_gamectrl:
        gc.dettach(ROBOT)
        gc.stop()
    controller.wait()
    print('\033[0m')

