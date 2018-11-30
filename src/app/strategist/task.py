#!/usr/bin/env python3
#coding: utf-8

import struct
from tcp import task_type, tcp_cmd_fmt, tcp_cmd_type, tcp_size

class task:
    def __init__(self, t=task_type.TASK_NONE, params=tuple()):
        self.__type = t
        self.__params = []
        self.__cmd = []
        self.__cmd.append(tcp_cmd_type.TASK_DATA)
        self.__cmd.append(True)
        for param in params:
            self.__params.append(param)

    def get_pack(self):
        fmt = tcp_cmd_fmt[tcp_cmd_type.TASK_DATA]
        idx = 0
        for p in self.__params:
            if type(p) == int:
                fmt += 'i'
            elif type(p) == float:
                fmt += 'f'
            elif type(p) == bool:
                fmt += '?'
            elif type(p) == str:
                fmt += '%ds'%len(p)
                self.__params[idx] = p.encode('utf-8')
            idx += 1
        self.__cmd.append(struct.calcsize(fmt)-tcp_size.head_size)
        self.__cmd.append(self.__type)
        for p in self.__params:
            self.__cmd.append(p)
        args = tuple(self.__cmd)
        return struct.pack(fmt, *args)
        