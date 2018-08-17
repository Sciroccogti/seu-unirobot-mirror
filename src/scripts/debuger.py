#!/usr/bin/env python

import sys
import common

debugers = {
        1: 'image_monitor',
        2: 'static_action',
        3: 'action_monitor',
        4: 'joint_revise',
        5: 'walk_remote'
    }

def print_help():
    print('\033[31mcommand: ./debuger.py debuger_id player_id\033[0m\n')
    print('\033[32m%5s\t%15s'%('id', 'debuger'))
    for key, value in debugers.items():
        print('%5s\t%15s'%(str(key), value))
    print('\033[0m')
        

if __name__ == '__main__':
    if not common.check_argv(sys.argv, 3):
        print_help()
        exit(1)
    
    try:
        if not common.check_id(sys.argv[2]):
            common.print_error('please check the robot id')
            exit(2)
        cmd = './%s -p%s'%(debugers[int(sys.argv[1])], sys.argv[2])
        common.run_cmd(cmd)
    except:
        print_help()
        exit(1)