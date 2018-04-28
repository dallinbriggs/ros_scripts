#!/bin/bash
echo "This script reindexes all of the .active rosbags within a directory."
echo "Are the bags in this directory? [y/n]"
read directory

if [ "$directory" == "y" ]; then
file_path="$PWD"
fi

if [ "$directory" == "n" ]; then
  echo "Enter the directory containing the bag files."
  read file_path
fi

cd $file_path

active_count="$(ls -1 *bag.active | wc -l)"
total_count="$(ls -1 *.bag* | wc -l)"

echo "Found $active_count active rosbags out of $total_count rosbags."
echo "Would you like to reindex them? [y/n]"
read REINDEX

if [ "$REINDEX" == "y" ]; then
  echo "Reindexing active rosbags."
  rosbag reindex *.bag.active
  wait
  echo "Rosbags have been reindexed."
fi

echo "Would you like to rename the .bag.active files to .bag files? [y/n]"
read rename

if [ "$rename" == "y" ]; then
  for file in *.bag.active
  do
    filename=$(basename "$file")
    filename="${filename%.*}"
    mv $file $filename
  done
fi
