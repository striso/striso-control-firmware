#!/bin/bash

if ! fmpp -q -C board.fmpp
then
  echo "aborted"
  exit 1
else
  git checkout ../board.mk
  echo "done"
  exit 0
fi
