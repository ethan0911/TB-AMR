#!/bin/bash
INPUTCELL=$1 #/usr/sci/data/ospray/exajet-d12/hexas.bin 
INPUTFIELD=$2  #velocity_magnitude.bin
OUTPUT=$3      #~/data/tamr/exajet/exajet

./ospRaw2Octree \
  -t exajet \
  -d $INPUTCELL \
  -f $INPUTFIELD \
  -o $OUTPUT
