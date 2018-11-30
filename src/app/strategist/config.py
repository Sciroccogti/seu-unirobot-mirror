#!/usr/bin/env python3
#coding: utf-8

import json

class config:
    _instance = None 
    
    def __new__(cls, *args, **kwargs): 
        if cls._instance is None: 
            cls._instance = super().__new__(cls,*args,**kwargs) 
        return cls._instance

    def __init__(self):
        self.id = 0
        self.__conf = None
    
    def init(self, id):
        self.id = id
        self.__conf = json.loads(self._get_json_from_conf('data/config.conf'))

    def get_config(self, key=''):
        conf = self.__conf
        keys = key.split('.')
        try:
            for k in keys:
                conf = conf.get(k)
            return conf
        except:
            return None

    def _get_json_from_conf(self, confname=''):
        json_data = ''
        for line in open(confname): 
            count_of_quotatuion = 0
            for c in line:
                if c == '\'' or c == '\"':
                    count_of_quotatuion += 1
                if c == '#' and count_of_quotatuion%2 == 0:
                    break
                json_data += c
        return json_data
