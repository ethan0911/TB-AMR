#!/bin/bash

INPUT=$1

./tamrViewer \
  -t synthetic \
  --use-tf-widget \
  -i $INPUT
