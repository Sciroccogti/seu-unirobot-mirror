#!/usr/bin/env python

import sys
import common


if __name__ == '__main__':
    if not common.check_argv(sys.argv):
        common.print_error('no enough arguments')
        exit(1)

    cmd = 'python exec.py %s poweroff'%sys.argv[1]
    common.run_cmd(cmd)