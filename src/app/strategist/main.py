#!/usr/bin/env python3
#coding: utf-8

import client
import time
import task
import robot
import tcp
import options
import sys
import signal
import gamecontroller
import teammate
import config

is_alive = False

def signal_handler(sig, frame):
    global is_alive
    if sig == signal.SIGINT:
        is_alive = False


if __name__ == '__main__':
    signal.signal(signal.SIGINT,signal_handler)
    opts = options.options()
    opts.init(sys.argv[1:])
    conf = config.config()
    conf.init(opts.id)

    rbt = robot.robot()
    cl = client.client(6666)
    cl.attach(rbt)
    cl.start()
    is_alive = True
    while not cl.connected and is_alive:
        time.sleep(0.5)
    if not cl.connected:
        cl.dettach(rbt)
        cl.stop()
        exit(0)
    
    if opts.use_gamectrl:
        gc = gamecontroller.gamecontroller(3838)
        gc.attach(rbt)
        gc.start()
    if opts.use_teammate:
        tm = teammate.teammate(4848)
        tm.attach(rbt)
        tm.start()
    cl.regsit(tcp.tcp_cmd_type.TASK_DATA, tcp.tcp_data_dir.DIR_SUPPLY)
    cl.regsit(tcp.tcp_cmd_type.WM_DATA, tcp.tcp_data_dir.DIR_APPLY)
    t1 = task.task(tcp.task_type.TASK_WALK, (0.04, 0.02, 20.0, True))
    data = t1.get_pack()
    t2 = task.task(tcp.task_type.TASK_ACT, ('reset',))
    data = t2.get_pack()
    while is_alive:
        if cl.connected:
            cl.send(data)
        else:
            break
        time.sleep(1)
    cl.dettach(rbt)
    cl.stop()
    if opts.use_teammate:
        tm.dettach(rbt)
        tm.stop()
    if opts.use_gamectrl:
        gc.dettach(rbt)
        gc.stop()

