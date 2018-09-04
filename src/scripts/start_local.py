#!/usr/bin/env python

import common
import sys
import config


if __name__ == '__main__':
    if not common.check_argv(sys.argv):
        common.print_error('no enough arguments')
        exit(1)

    robot_id = sys.argv[1]
    if not common.check_id(robot_id):
        common.print_error('please check the robot id')
        exit(2)

    if not common.build_project(True):
        common.print_error('build error, please check code')
        exit(3)

    args = common.parse_argv(sys.argv)

    cmd = 'cd %s/bin; ./%s %s '%(config.project_dir, config.exec_file_name, args)
    common.run_cmd(cmd)