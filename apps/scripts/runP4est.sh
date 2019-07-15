#!/bin/bash

INPUT=$1

./p4estViewer \
  -t p4est \
  -i $INPUT \
  --use-tf-widget
