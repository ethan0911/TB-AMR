#!/bin/bash
INPUTFIELD=$1
OUTPUT=$2

./ospRaw2Octree \
  -t landing \
  -d /usr/sci/data/ospray/ExaBrick/landinggear/chombo.exa.cells \
  -f $INPUTFIELD \
  -o $OUTPUT
