#!/bin/bash

CURDIR=`pwd`

A=`whoami`
B=`getconf LONG_BIT`

if [ $A != 'root' ]; then
   echo "You have to be root to run this script"
   echo "Fail !!!"
   exit 1;
fi


cp 88-mvusb.rules /etc/udev/rules.d/

echo "Successful"
echo "Please  restart system  now!!!"
