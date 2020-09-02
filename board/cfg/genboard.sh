#!/bin/bash

if ! fmpp -q -C board.fmpp
then
  echo
  echo "aborted"
  exit 1
else
  git checkout ../board.mk
  echo
  echo "done"
  exit 0
fi
