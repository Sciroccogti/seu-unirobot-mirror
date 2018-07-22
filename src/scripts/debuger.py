#!/usr/bin/env python

import sys
import common

if __name__ == '__main__':
    debugers = {
        'im': 'imageMonitor',
        'is': 'imageSamples'
    }
    print('''how to use
    function: run debuger in local machine
    example: python exec_local.py id debuger argv
    ''')
    print('abbreviation of debuger')
    print(debugers)
    if not common.check_argv(sys.argv, 3):
        common.print_error('no enough arguments')
        exit(1)

    args = common.parse_argv(sys.argv, 3)
    args += ' -d0'
    cmd = './%s %s'%(debugers[sys.argv[2], args])
    common.run_cmd(cmd)