#!/usr/bin/env python

import config
import os
import shutil
import json
import common


if __name__ == '__main__':
    offset_file = ''
    with open(config.config_file_name,'r') as f:
        conf = json.load(f)
    try:
        players =  conf.get('players')
        player = players.get(id)
        offset_file = player.get('offset_file')
    except:
        common.print_error('get offset file error')
        exit(1)
    new_offset_file = config.project_dir + '/bin/' + offset_file
    origin_offset_file = config.project_dir + '/src/' + offset_file
    os.rename(origin_offset_file, origin_offset_file+'.bak')
    shutil.copyfile(new_offset_file, origin_offset_file)
