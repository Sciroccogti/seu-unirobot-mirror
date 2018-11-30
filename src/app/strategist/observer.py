#!/usr/bin/env python3
#coding: utf-8

class publisher:
    def __init__(self):
        self.__subs = []

    def attach(self, sub):
        self.__subs.append(sub)

    def dettach(self, sub):
        self.__subs.remove(sub)

    def notify(self, data=tuple()):
        for sub in self.__subs:
            sub.update(self, data)


class subscriber:
    def __init__(self):
        pass

    def update(self, pub, data=tuple()):
        pass