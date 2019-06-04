#!/bin/bash
PREFIX=$1

./p4estViewer \
  -t exajet \
  -d /usr/sci/data/ospray/exajet-d12/hexas.bin \
  -f $PREFIX \
  /usr/sci/data/ospray/exajet-d12/surfaces_vtp/*.vtp
