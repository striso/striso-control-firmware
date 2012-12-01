#!/bin/sh
DEV=$1

stty -F $DEV clocal cread

# empty buffer
dd bs=64 if=$DEV of=/dev/null &
pid=$!
sleep .1
kill $pid

# do transfer speed test
dd bs=64 if=$DEV of=/dev/null &
pid=$!
sleep 1
kill -USR1 $pid
sleep .1
kill $pid
