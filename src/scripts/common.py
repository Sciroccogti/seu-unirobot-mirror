#!/usr/bin/env python3
#coding: utf-8

import subprocess
import json
import config
import os
import hashlib


def get_json_from_conf(confname=''):
    json_data = ''
    for line in open(confname): 
        count_of_quotatuion = 0
        for c in line:
            if c == '\'' or c == '\"':
                count_of_quotatuion += 1
            if c == '#' and count_of_quotatuion%2 == 0:
                break
            json_data += c
    return json_data


def run_cmd(cmd, prt=True):
    p = subprocess.Popen(cmd, shell=True)
    p.wait()
    if p.poll() == 0:
        return True
    else:
        return False


def check_id(id):
    conf = json.loads(get_json_from_conf(config.config_file_name))
    players =  conf.get('players')
    player = players.get(id)
    if player is None:
        return False
    else:
        return True


def get_ip(id):
    conf = json.loads(get_json_from_conf(config.config_file_name))
    try:
        players =  conf.get('players')
        player = players.get(id)
        return player.get('address')
    except:
        return None


def check_net(ip):
    backinfo = os.system('ping -c 1 -w 1 %s'%ip)
    if not backinfo:
        return True
    else:
        return False
    
    
def get_config(key=''):
    conf = json.loads(get_json_from_conf(config.config_file_name))
    keys = key.split('.')
    try:
        for k in keys:
            conf = conf.get(k)
        return conf
    except:
        return None


def check_weight():
    net_file = '%s/bin/%s/%s'%(config.project_dir, config.target_dir, get_config('net_weights_file'))
    md5_file = '%s/bin/%s'%(config.project_dir, config.weights_md5_file)
    f_md5 = hashlib.md5(open(net_file, 'rb').read()).hexdigest()
    if not os.path.exists(md5_file):
        f = open(md5_file, 'w')
        f.write(f_md5)
        f.close()
        return False
    with open(md5_file, 'r') as md5f:
        md5_code = md5f.read().strip()
    if md5_code != f_md5:
        with open(md5_file, 'w') as md5f:
            md5f.write(f_md5)
        return False
    return True


def build_project(cross, j=2):
    if cross:
        build_dir = '%s/%s-build'%(config.project_dir, config.target_dir)
        if not os.path.exists(build_dir):
            os.mkdir(build_dir)
        cmd = 'rm -rf %s/bin/%s; cd %s; cmake -D CROSS=ON ..; make install -j%d'\
            %(config.project_dir, config.target_dir, build_dir, j)
    else:
        build_dir = '%s/x86_64-build'%config.project_dir
        if not os.path.exists(build_dir):
            os.mkdir(build_dir)
        cmd = 'cd %s; cmake ..; make install -j%d'%(build_dir, j)
    return run_cmd(cmd)


def compress_files():
    if check_weight():
        os.remove('%s/bin/%s/%s'%(config.project_dir, config.target_dir, get_config('net_weights_file')))
    cmd = 'cd %s/bin; tar zcf %s %s'%(config.project_dir, config.compress_file_name, config.target_dir)
    return run_cmd(cmd, False)


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


def print_error(err):
    print('\033[1;31m %s \033[0m'%err)


def print_info(info):
    print('\033[1;32m %s \033[0m'%info)
