#!/bin/bash

INPUT=$1

./tamrViewer \
  -t p4est \
  -i $INPUT \
  --use-tf-widget
