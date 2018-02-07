#!/bin/bash
echo Enter the directory containing the bag files. If here, press enter.
read DIRECTORY
if [ -z DIRECTORY ]
  DIRECTORY=$pwd
echo $active_count rosbags out of $total_count need to be reindexed.
echo Would you like to reindex them all at once?
echo $
