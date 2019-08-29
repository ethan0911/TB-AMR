#!/bin/bash

OUTPUT=$1

./ospRaw2Octree \
  -t p4est \
  -d ~/work/ospray_2_0/ospray/modules/amr_project/data/testcube/p4est-viz-06.p4est \
  -o $OUTPUT
