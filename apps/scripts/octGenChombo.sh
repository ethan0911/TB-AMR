#!/bin/bash
INPUTCELL=$1
INPUTFIELD=$2
OUTPUT=$3

./ospRaw2Octree \
  -t landing \
  -d $INPUTCELL \
  -f $INPUTFIELD \
  -o $OUTPUT
