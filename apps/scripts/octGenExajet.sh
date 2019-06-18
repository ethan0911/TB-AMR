#!/bin/bash
INPUTFIELD=$1
OUTPUT=$2

./ospRaw2Octree \
  -t exajet \
  -d /usr/sci/data/ospray/exajet-d12/hexas.bin \
  -f $INPUTFIELD \
  -o $OUTPUT
