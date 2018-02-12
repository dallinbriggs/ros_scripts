#!/bin/bash
echo "This script reindexes all of the .active rosbags within a directory."
echo "Are the bags in this directory? [y/n]"
read DIRECTORY

if [ "$DIRECTORY" == "y" ]; then
  PATH="$PWD"
  echo "path is $PATH"
fi

if [ "$DIRECTORY" == "n" ]; then
  echo "Enter the directory containing the bag files."
  read PATH
fi

{ cd $PATH } || { echo "Invalid path. Enter the correct directory."

active_count="$(ls -1 *bag.active | wc -l)"
total_count="$(ls -1 *.bag* | wc -l)"

echo "Found $active_count active rosbags out of $total_count rosbags."
echo "Would you like to reindex them? [y/n]"
read REINDEX

if [ "$REINDEX" == "y" ]; then
  echo "Reindexing active rosbags."
  rosbag reindex *.bag.active
fi

wait

echo "Rosbags have been reindexed."
echo "Would you like to rename the .bag.active files to .bag files? [y/n]"
read RENAME

if [ "$RENAME" == "y" ]; then

fi
