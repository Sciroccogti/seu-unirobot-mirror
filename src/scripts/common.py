#!/usr/bin/env python

import subprocess
import json
import config

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


def print_error(err):
    print('\033[1;31m %s \033[0m'%err)


def print_info(info):
    print('\033[1;32m %s \033[0m'%info)
