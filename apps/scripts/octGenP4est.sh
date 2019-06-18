#!/bin/bash

OUTPUT=$1

./ospRaw2Octree \
  -t p4est \
  -d ~/work/ospray_2_0/ospray/modules/amr_project/data/cube_with_fe_data/p4est-withdata-06 \
  -o $OUTPUT