function [ data, errors, warnings, infos, error_time, warning_time, info_time ] = processAllTopics( bagfile )

assert(exist(bagfile,'file') ~= 0,'Bag file does not exist');

clear rosbag_wrapper;
clear ros.Bag;

addpath('matlab_rosbag-0.4.1-linux64')

bag = ros.Bag.load(bagfile);
% keyboard;
[data, errors, warnings, infos, error_time, warning_time, info_time] = processTopics(bag.topics,bagfile);
end

