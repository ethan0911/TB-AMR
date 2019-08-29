#!/bin/bash

DATA_TYPE=$1
INPUT=$2

PARAMETERS=${@:3}

if [ $DATA_TYPE == 'exajet' ]
then 
  if [[ $PARAMETERS != *"-f "* ]] && [[ $PARAMETERS != *"-field"* ]]
  then
    echo "A field name must be specified for NASA exajet data!"
    exit
  fi
fi

#amplxe-cl -collect hotspots \
  ./tamrViewer \
  -t $DATA_TYPE \
  -i $INPUT \
  ${@:3}
