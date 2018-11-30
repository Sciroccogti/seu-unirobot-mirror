#!/usr/bin/env python3
#coding: utf-8

import socket
import threading
import observer

class udp_recver(threading.Thread, observer.publisher):
    def __init__(self, port):
        threading.Thread.__init__(self)
        self.__port = port
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.alive = False

    def stop(self):
        self.alive = False

    def run(self):
        try:
            self.__sock.bind(('', self.__port))
            self.alive = True
        except:
            pass
        while self.alive:
            try:
                data, addr = self.__sock.recvfrom(1024)
                self.notify()
            except:
                pass
    


class udp_sender:
    def __init__(self, port):
        self.__port = port
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, data):
        self.__sock.sendto(data, ('', self.__port))