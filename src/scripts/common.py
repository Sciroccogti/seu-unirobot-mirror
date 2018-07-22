#!/usr/bin/env python

import os
import subprocess
import json
import config
import paramiko

def run_cmd(cmd):
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    while p.poll() is None:
        print(p.stdout.readline())
    if p.poll() == 0:
        return True
    else:
        return False

def build_project():
    cmd = 'cd %s/build; make install'%config.project_dir
    return run_cmd(cmd)

def check_id(id):
    with open(config.config_file_name,'r') as f:
        conf = json.load(f)
    players =  conf.get('players')
    player = players.get(id)
    if player is None:
        return False
    else:
        return True
    
def get_ip(id):
    with open(config.config_file_name,'r') as f:
        conf = json.load(f)
    try:
        players =  conf.get('players')
        player = players.get(id)
        return player.get('address')
    except:
        return None
    
def compress_files():
    cmd = 'cd %s/bin; tar zcvf %s %s data'%(config.project_dir, config.compress_file_name, config.exec_file_name)
    return run_cmd(cmd)
    
def parse_argv(argv=[], start=2):
    args = ' -p'+argv[1]
    for i in range(start,len(argv)):
        args += (' '+argv[i])
    return args

def check_argv(argv=[], num=2):
    if len(argv) < num:
        return False
    else:
        return True

def print_error(info):
    print('\033[1;31m %s \033[0m'%info)
    
