#!/bin/bash

args=("$@")

if [ "${args[0]}" = "" ]; then
    echo required test name
    exit
fi

if [ "$HOSTNAME" = "usernet-vm1-amd" ]; then
    LD_PRELOAD="./libusernet.so /usr/lib/x86_64-linux-gnu/libsyscall_intercept.so" ./server 8864
    exit
fi

host="Hello"
if [ "$HOSTNAME" = "usernet-vm2-intel" ]; then
    host='192.168.1.101'
else
    host='192.168.10.101'
fi

i=0
while [ "$i" -lt 100 ]
do
    # echo $(hostname)
    LD_PRELOAD="./libusernet.so /usr/lib/x86_64-linux-gnu/libsyscall_intercept.so" ./client $host 8864 TCP_STREAM 5 >> test.stream.$HOSTNAME.${args[0]}.log
    LD_PRELOAD="./libusernet.so /usr/lib/x86_64-linux-gnu/libsyscall_intercept.so" ./client $host 8864 TCP_RR 5 >> test.rr.$HOSTNAME.${args[0]}.log
    
    echo $i
    i=$(expr $i + 1)
done
