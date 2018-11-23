#!/usr/bin/env python

import os
import sys

if __name__ == '__main__': 
    build_dir = 'aarch64-build'
    t = '2'
    if len(sys.argv) == 2:
        t = sys.argv[1]
    if not os.path.exists(build_dir):
        os.mkdir(build_dir)
    cmd = 'cd %s; cmake -D CROSS=ON ..; make install -j%s'%(build_dir, t);
    os.system(cmd)
