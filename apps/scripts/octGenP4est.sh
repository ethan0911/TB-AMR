#!/bin/bash

OUTPUT=$1
INPUT=$2

./ospRaw2Octree \
  -t p4est \
  -d $INPUT \
  -o $OUTPUT
