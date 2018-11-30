#!/usr/bin/env python3
#coding: utf-8

import getopt

class options:
    _instance = None 

    def __new__(cls, *args, **kwargs): 
        if cls._instance is None: 
            cls._instance = super().__new__(cls,*args,**kwargs) 
        return cls._instance

    def __init__(self):
        self.use_gamectrl = False
        self.use_teammate = False
        self.id = 0
        
    def init(self, argv):
        try:
            opts, args = getopt.getopt(argv, 'hp:g:t:')
            for opt, arg in opts:
                if opt == '-h':
                    print('''usage: main.py -g <0|1> -t <0|1>\n-g: 0. do not use gamectrl; 1. use gamectrl\n-t: 0. do not use teammate; 1. use teammate\n''')
                    exit(1)
                elif opt == '-g':
                    self.use_gamectrl = bool(arg)
                elif opt == '-t':
                    self.use_teammate = bool(arg)
                elif opt == '-p':
                    self.id = int(arg)             
        except getopt.GetoptError as e:
            print(e.msg)
            exit(1)
