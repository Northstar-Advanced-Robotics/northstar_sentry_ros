#!/bin/bash

HERE=$(dirname ${BASH_SOURCE[0]})

if (("$#" != 1)); then 
  echo "You need to provide a bag name"  
  exit 1
fi

ros2 bag record -s mcap --storage-preset-profile zstd_fast $(cat ${HERE}/record_video_topics.txt) -o ${HERE}/../bags/"$1" 
