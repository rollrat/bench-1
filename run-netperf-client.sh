#!/bin/bash

args=("$@")

if [ "${args[0]}" = "" ]; then
    echo required test name
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
    ./client $host 8864 TCP_STREAM 5 >> test.stream.$HOSTNAME.${args[0]}.log
    ./client $host 8864 TCP_RR 5 >> test.rr.$HOSTNAME.${args[0]}.log
    
    i=$(expr $i + 1)
done
