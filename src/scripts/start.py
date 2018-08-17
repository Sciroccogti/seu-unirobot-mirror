#!/usr/bin/env python

import os
import common
import config
import SSHConnection
import json


if __name__ == '__main__': 
    jdata = common.get_json_from_conf("data/actuator.conf")
    conf = json.loads(jdata)
    print(conf.get('team_name'))