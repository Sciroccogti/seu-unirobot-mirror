#!/usr/bin/env python3
#coding: utf-8

import os
import re

project_name = 'seu-unirobot'
project_dir = re.findall('(.*)%s'%project_name, os.path.realpath(__file__))[0] + project_name
target_dir = 'controller'
config_file_name = project_dir + '/src/data/config.conf'
compress_file_name = target_dir+'.tar.gz'
exec_file_name = 'controller'
username = 'root'
password = 'nvidia'
ssh_port = 22
remote_dir = '/opt/'
start_up_file = '/etc/rc.local'
wan_file = '/etc/NetworkManager/system-connections/SEU-RoboCup'
lan_file = '/etc/NetworkManager/system-connections/static'
weights_md5_file = 'weights_md5.txt'
