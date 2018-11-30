#!/usr/bin/env python3
#coding: utf-8

import socket
import threading
import time
import struct
import observer
from tcp import tcp_cmd_type, tcp_cmd_fmt, tcp_size

class client(observer.publisher):
    def __init__(self, port, addr='127.0.0.1'):
        observer.publisher.__init__(self)
        self.__addr = addr
        self.__port = port
        self.__sock = socket.socket()
        self.__td = None
        self.connected = False
        self.is_alive = False

    def start(self):
        self.__sock.setblocking(True)
        self.__sock.settimeout(1.0)
        self.is_alive = True
        self.__td = threading.Thread(target=self.run)
        self.__td.start()
        return True

    def stop(self):
        self.is_alive = False

    def send(self, data):
        self.__sock.send(data)

    def regsit(self, t, d):
        cmd = []
        cmd.append(tcp_cmd_type.REG_DATA)
        cmd.append(True)
        cmd.append(tcp_size.int_size+tcp_size.int_size)
        cmd.append(t)
        cmd.append(d)
        fmt = '=i?Iii'
        data = struct.pack(fmt, *(tuple(cmd)))
        self.send(data)

    def run(self):
        while self.is_alive:
            try:
                self.__sock.connect((self.__addr, self.__port))
                self.connected = True
                break
            except ConnectionError as e:
                time.sleep(1)
        while self.is_alive:
            try:
                data = self.__sock.recv(256)
                if len(data) == 0:
                    self.__sock.close()
                    self.connected = False
                    self.is_alive = False
                    print('The main controller is closed!')
                else:
                    (cmd_type,) = struct.unpack('=i',data[:4])
                    res = tuple()
                    if cmd_type == tcp_cmd_type.WM_DATA:
                        res = struct.unpack(tcp_cmd_fmt[tcp_cmd_type.WM_DATA], data)[3:]
                    self.notify(res)
            except:
                pass
