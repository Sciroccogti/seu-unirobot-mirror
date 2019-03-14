#!/usr/bin/env python3
#coding: utf-8

import os
import socket
from ssh2.session import Session
from ssh2.utils import wait_socket
from ssh2.fileinfo import FileInfo
import ssh2

class ssh_connection:
    def __init__(self, host, port, username, password):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((host, port))
        self._session = Session()
        self._session.handshake(self._sock)
        self._session.userauth_password(username, password)
        self._session.set_blocking(True)

    def exec_command(self, command):
        channel = self._session.open_session()
        channel.execute(command)
        try:
            size, data = channel.read()
            while size > 0:
                print(data.decode('utf-8'))
                size, data = channel.read()
        except: 
            pass
        size, data = channel.read()
        print(data.decode('utf-8'))
        channel.close() 

    def upload(self, local, remote):
        fileinfo = os.stat(local)
        chan = self._session.scp_send64(remote, fileinfo.st_mode & 0o777, fileinfo.st_size, \
            fileinfo.st_mtime, fileinfo.st_atime)
        with open(local, 'rb') as local_fh:
            l = 0
            for data in local_fh:
                chan.write(data)
                l = l+len(data)
                print('\rupload                                        {0}%'\
                    .format(int(l/fileinfo.st_size*100)), end='', flush=True)
            print('\n')

    def download(self, remote, local):
        chan, fileinfo = self._session.scp_recv2(remote)
        with open(local, 'rb') as local_fh::
            size, data = chan.read()
            while size>0:
                local_fh.write(data)
                size, data = chan.read()

    def close(self):
        self._session.disconnect()
        self._sock.close()