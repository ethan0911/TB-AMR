#!/bin/bash

INPUT=$1

./p4estViewer \
  -t synthetic \
  --use-tf-widget \
  -i $INPUT
