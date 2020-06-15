#!/bin/bash

if ! fmpp -q -C board.fmpp
then
  echo
  echo "aborted"
  exit 1
else
  echo
  echo "done"
  exit 0
fi
