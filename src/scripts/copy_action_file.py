#!/usr/bin/env python

import config
import os
import shutil
import json
import common


if __name__ == '__main__':
    action_file = ''
    with open(config.config_file_name,'r') as f:
        conf = json.load(f)
    try:
        players =  conf.get('players')
        player = players.get(id)
        action_file = player.get('action_file')
    except:
        common.print_error('get action file error')
        exit(1)
    new_action_file = config.project_dir + '/bin/' + action_file
    origin_action_file = config.project_dir + '/src/' + action_file
    os.rename(origin_action_file, origin_action_file+'.bak')
    shutil.copyfile(new_action_file, origin_action_file)
