#!/usr/bin/env python

import os
import re

project_name = 'seu-unirobot'
project_dir = re.findall('(.*)%s'%project_name, os.path.realpath(__file__))[0] + project_name

config_file_name = project_dir + '/src/data/config.conf'
compress_file_name = 'run.tar.gz'
exec_file_name = 'controller'
username = 'root'
password = 'nvidia'
ssh_port = 22
remote_dir = '/opt/'
start_up_file = '/etc/rc.local'
wifi_script = 'generate_wifi.py'
