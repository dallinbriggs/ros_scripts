#!/bin/bash
echo "This script reindexes all of the .active rosbags within a directory."
echo "Are the bags in this directory? [y/n]"
read DIRECTORY

if [ "$DIRECTORY" == "n" ]; then
  echo "Enter the directory containing the bag files."
  read DIRECTORY
  echo "TODO 1"
fi

if [ "$DIRECTORY" == "y" ]; then
  echo "Reindexing bags in this directory."
  rosbag reindex *.bag.active
fi

for []

echo $active_count rosbags out of $total_count need to be reindexed.
echo Would you like to reindex them all at once?
echo $
