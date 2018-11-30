#!/usr/bin/env python3
#coding: utf-8

import observer
import client
import gamecontroller
import teammate
import threading

class robot(observer.subscriber):
    _instance = None 
    
    def __new__(cls, *args, **kwargs): 
        if cls._instance is None: 
            cls._instance = super().__new__(cls,*args,**kwargs) 
        return cls._instance

    def __init__(self):
        observer.subscriber.__init__(self)
        self.wm_mtx = threading.Lock()

    def update(self, pub, data=tuple()):
        print(data)