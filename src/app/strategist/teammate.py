#!/usr/bin/env python3
#coding: utf-8

import socket
import observer
import threading

class teammate(observer.publisher):
    def __init__(self, port):
        observer.publisher.__init__(self)
        self.__port = port
        self.__td = None
        self.is_alive = False
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def __del__(self):
        self.__td.join()

    def start(self):
        try:
            self.__sock.bind(('', self.__port))
            self.alive = True
        except:
            return False
        self.is_alive = True
        self.__td = threading.Thread(target=self.run)
        self.__td.start()
        return True

    def stop(self):
        self.is_alive = False
        self.__sock.close()
    
    def run(self):
        while self.alive:
            try:
                data, addr = self.__sock.recvfrom(1024)
                self.notify()
            except:
                pass


