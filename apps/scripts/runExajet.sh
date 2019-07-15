#!/bin/bash
INPUT=$1

./p4estViewer \
  -t exajet \
  --use-tf-widget \
  -i $INPUT \
  /usr/sci/data/ospray/exajet-d12/surfaces_vtp/*.vtp
