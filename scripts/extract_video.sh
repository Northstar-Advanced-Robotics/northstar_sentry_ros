#!/bin/bash
# 
HERE=$(dirname ${BASH_SOURCE[0]})

if (("$#" != 2)); then 
  echo "You need to provide a bag name and a camera"  
  exit 1
fi

ros2 bag convert -i "$1" -o ${HERE}/decompress.yaml
ros2 bag to_video uncompressed_bag --fps 30 -t /"$2"/camera/color/image_raw -o ${HERE}/../videos/"$1-$2".mp4
rm -r uncompressed_bag
