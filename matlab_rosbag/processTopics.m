function [data, errors, warnings, infos, error_time, warning_time, info_time] = processTopics(topics,bagfile,t0)
%clear rosbag_wrapper;
%clear ros.Bag;
% keyboard;
if nargin < 3
    t0 = -1;
end


addpath('./matlab_rosbag-0.4.1-linux64')
% addpath('./navfn')

bag = ros.Bag.load(bagfile);
for topic = topics
    ind = find(ismember(bag.topics,topic),1);
    if isempty(ind)
        fprintf('Could not find topic: %s\n',topic{:});
        continue;
    end
    fprintf('   Processing topic: %s\n',topic{:});
    type = bag.topicType(topic{:});
    [msgs, meta] = bag.readAll(topic);
    
    a = [msgs{:,:}];
    c = [meta{:,:}];
    d = [c.time];
    if t0 < 0
        t0 = d(1).time - 0.1;
    else if (t0 > 0) && (t0 < 10000)
            t0 = d(1).time - 0.1 - t0;
        end
    end
    clear struct
    switch type{:}
        case 'sensor_msgs/NavSatFix'
%             keyboard;
            struct.status = [a.status];
            struct.latitude = [a.latitude];
            struct.longitude = [a.longitude];
            struct.altitude = [a.altitude];
            struct.position_covariance = [a.position_covariance];
            struct.time = [d.time] - t0;
            
        case 'std_msgs/Float64'
%             keyboard;
            struct.heading = [a];
            struct.time = [d.time] - t0;
        case 'geometry_msgs/TwistStamped'
%             keyboard;
            struct.twist = [a.twist];
            struct.time = [d.time] - t0;
            
        case 'mavros_msgs/HomePosition'
%             keyboard;
            struct.geo = [a.geo];
            struct.position = [a.position];
            struct.orientation = [a.orientation];
            struct.approach = [a.approach];
            struct.time = [d.time] - t0;
            
        case 'mavros_msgs/VFR_HUD'
%             keyboard;
            struct.airspeed = [a.airspeed];
            struct.groundspeed = [a.groundspeed];
            struct.heading = [a.heading];
            struct.throttle = [a.throttle];
            struct.altitude = [a.altitude];
            struct.climb = [a.climb];
            struct.time = [d.time] - t0;
            
        case 'mavros_msgs/RCIn'
            struct.rssi = [a.rssi];
            struct.channels = [a.channels];
            struct.time = [d.time] - t0;
            
        case 'mavros_msgs/RCOut'
            struct.channels = [a.channels];
            struct.time = [d.time] - t0;
            
        case 'mavros_msgs/State'
            struct.connected = [a.connected];
            struct.armed = [a.armed];
            struct.guided = [a.guided];
            temp = struct2cell(a);
            struct.mode = squeeze(temp(5,1,:));
            struct.system_status = [a.system_status];
            struct.time = [d.time] - t0;
            clear temp;
            
        case 'mavros_msgs/WaypointReached'
            struct.wp_seq = [a.wp_seq];
            struct.time = [d.time] - t0;         
            
        case 'mavros_msgs/WaypointList'
            struct.current_seq = [a.current_seq];
            struct.waypoints = [a.waypoints];
            struct.time = [d.time] - t0;
            
        case 'geometry_msgs/QuaternionStamped'
            struct.quaternion = [a.quaternion];
            struct.time = [d.time] - t0;
            
        case 'std_msgs/Float32'
            struct.value = [a];
            struct.time = [d.time] - t0;
            
        case 'gimbal_serializer/status'
            struct.command_in_Hz = [a.command_in_Hz];
            struct.servo_command_Hz = [a.servo_command_Hz];
            struct.roll_command = [a.roll_command];
            struct.pitch_command = [a.pitch_command];
            struct.yaw_command = [a.yaw_command];
            struct.time = [d.time] - t0;
            
        case 'geometry_msgs/Vector3'
%             keyboard;
            struct.vector = [a];
            struct.time = [d.time] - t0;
            
        case 'rosplane_msgs/Current_Path'
%             keyboard;
            struct.flag = [a.flag];
            struct.Va_d = [a.Va_d];
            struct.r = [a.r];
            struct.q = [a.q];
            struct.c = [a.c];
            struct.rho = [a.rho];
            struct.lambda = [a.lambda];
            struct.time = [d.time] - t0;
            %         case 'rosflight_msgs/Command'
            %             keyboard;
        case 'rosplane_msgs/Controller_Commands'
            struct.Va_c = [a.Va_c];
            struct.h_c = [a.h_c];
            struct.chi_c = [a.chi_c];
            struct.phi_ff = [a.phi_ff];
            struct.aux = [a.aux];
            struct.aux_valid = [a.aux_valid];
            struct.time = [d.time] - t0;
        case 'rosplane_msgs/Controller_Internals'
            struct.theta_c = [a.theta_c];
            struct.phi_c = [a.phi_c];
            struct.alt_zone = [a.alt_zone];
            struct.aux = [a.aux];
            struct.aux_valid = [a.aux_valid];
            struct.ZONE_TAKE_OFF = [a.ZONE_TAKE_OFF];
            struct.ZONE_CLIMB = [a.ZONE_CLIMB];
            struct.ZONE_DESEND = [a.ZONE_DESEND];
            struct.ZONE_ALTITUDE_HOLD = [a.ZONE_ALTITUDE_HOLD];
            struct.time = [d.time] - t0;
        case 'rosgraph_msgs/Log'
            %             keyboard;
            b = {a.msg};
            time = [d.time] - t0;
            time_cell = num2cell(time);
            errors = {b{[a.level] == 8}};
            warnings = {b{[a.level] == 4}};
            infos = {b{[a.level] == 2 }};
            error_time = {time_cell{[a.level] == 8}};
            warning_time = {time_cell{[a.level] == 4}};
            info_time = {time_cell{[a.level] == 2}};
            struct.msg = [a.msg];
        case 'relative_nav_msgs/FilterState'
            b = [a.transform];
            struct.transform.translation = [b.translation];
            struct.transform.rotation = [b.rotation];
            [r,p,y] = rollPitchYawFromQuaternion(struct.transform.rotation.');
            struct.transform.euler = [r,p,y]*180/pi;
            struct.velocity = [a.velocity];
            struct.node_id = [a.node_id];
            struct.time = [d.time] - t0;
        case 'rosplane_msgs/State'
            struct.position = [a.position];
            struct.Va = [a.Va];
            struct.alpha = [a.alpha];
            struct.beta = [a.beta];
            struct.phi = [a.phi];
            struct.theta = [a.theta];
            struct.psi = [a.psi];
            struct.chi = [a.chi];
            struct.p = [a.p];
            struct.q = [a.q];
            struct.r = [a.r];
            struct.Vg = [a.Vg];
            struct.wn = [a.wn];
            struct.we = [a.we];
            struct.quat = [a.quat];
            struct.quat_valid = [a.quat_valid];
            struct.chi_deg = [a.chi_deg];
            struct.psi_deg = [a.psi_deg];
            struct.time = [d.time] - t0;
        case 'rosflight_msgs/State'
            struct.position = [a.position];
            struct.Va = [a.Va];
            struct.alpha = [a.alpha];
            struct.beta = [a.beta];
            struct.phi = [a.phi];
            struct.theta = [a.theta];
            struct.psi = [a.psi];
            struct.chi = [a.chi];
            struct.p = [a.p];
            struct.q = [a.q];
            struct.r = [a.r];
            struct.Vg = [a.Vg];
            struct.wn = [a.wn];
            struct.we = [a.we];
            struct.quat = [a.quat];
            struct.quat_valid = [a.quat_valid];
            struct.chi_deg = [a.chi_deg];
            struct.psi_deg = [a.psi_deg];
            struct.time = [d.time] - t0;
        case 'rosflight_msgs/OutputRaw'
            struct.values = [a.values];
            struct.time = [d.time] - t0;
        case 'geometry_msgs/Vector3Stamped'
            struct.vector = [a.vector];
            struct.time = [d.time] - t0;
        case 'rosflight_msgs/Airspeed'
            struct.velocity = [a.velocity];
            struct.differential_pressure = [a.differential_pressure];
            struct.temperature = [a.temperature];
            struct.time = [d.time] - t0;
        case 'rosflight_msgs/Attitude'
            struct.attitude = [a.attitude];
            struct.angular_velocity = [a.angular_velocity];
            struct.time = [d.time] - t0;
        case 'rosflight_msgs/Barometer'
            struct.altitude = [a.altitude];
            struct.pressure = [a.pressure];
            struct.temperature = [a.temperature];
            struct.time = [d.time] - t0;
        case 'sensor_msgs/Temperature'
            struct.temperature = [a.temperature];
            struct.variance = [a.variance];
            struct.time = [d.time] - t0;
        case 'sensor_msgs/MagneticField'
            struct.magnetic_field = [a.magnetic_field];
            struct.magnetic_field_covariance = [a.magnetic_field_covariance];
            struct.time = [d.time] - t0;
        case 'sensor_msgs/FluidPressure'
            struct.fluid_pressure = [a.fluid_pressure];
            struct.variance = [a.variance];
            struct.time = [d.time] - t0;
        case 'inertial_sense/GPS'
            struct.num_sat = [a.num_sat];
            struct.fix_type = [a.fix_type];
            struct.cno = [a.cno];
            struct.latitude = [a.latitude];
            struct.longitude = [a.longitude];
            struct.altitude = [a.altitude];
            struct.hMSL = [a.hMSL];
            struct.hAcc = [a.hAcc];
            struct.vAcc = [a.vAcc];
            struct.pDop = [a.pDop];
            struct.linear_velocity = [a.linear_velocity];
            struct.sAcc = [a.sAcc];
            struct.hAcc = [a.hAcc];
            struct.vAcc = [a.vAcc];
            struct.time = [d.time] - t0;
        case 'rosflight_msgs/RCRaw'
            struct.values = [a.values];
            struct.time = [d.time] - t0;
        case 'rosflight_msgs/Status'
            struct.armed = [a.armed];
            struct.failsafe = [a.failsafe];
            struct.rc_override = [a.rc_override];
            struct.num_errors = [a.num_errors];
            struct.loop_time_us = [a.loop_time_us];
            struct.time = [d.time] - t0;
        case 'rosflight_msgs/GPS'
            struct.fix = [a.fix];
            struct.NumSat = [a.NumSat];
            struct.latitude = [a.latitude];
            struct.longitude = [a.longitude];
            struct.altitude = [a.altitude];
            struct.speed = [a.speed];
            struct.ground_course = [a.ground_course];
            struct.covariance = [a.covariance];
            struct.time = [d.time] - t0;
%         case 'sensor_msgs/NavSatFix'
%             struct.latitude = [a.latitude];
%             struct.longitude = [a.longitude];
%             cov = [a.position_covariance];
%             struct.hAcc = cov(1,:);
%             struct.time = [d.time] - t0;
        case 'geometry_msgs/Transform'
            struct.transform.translation = [a.translation];
            struct.transform.rotation = [a.rotation];
            [r,p,y] = rollPitchYawFromQuaternion(struct.transform.rotation.');
            struct.transform.euler = [r,p,y]*180/pi;
            struct.time = [d.time] - t0;
        case 'geometry_msgs/TransformStamped'
            b = [a.transform];
            struct.transform.translation = [b.translation];
            struct.transform.rotation = [b.rotation];
            [r,p,y] = rollPitchYawFromQuaternion(struct.transform.rotation.');
            struct.transform.euler = [r,p,y]*180/pi;
            struct.time = [d.time] - t0;
        case 'relative_nav_msgs/DesiredState'
            struct.pose = [a.pose];
            struct.velocity = [a.velocity];
            struct.acceleration = [a.acceleration];
            struct.node_id = [a.node_id];
            
            struct.time = [d.time] - t0;
        case 'relative_nav_msgs/Snapshot'
            struct.state = [a.state];
            struct.covariance = [a.covariance_diagonal];
            struct.node_id = [a.node_id];
            struct.time = [d.time] - t0; % This could also use the header time
        case 'geometry_msgs/PoseStamped'
            b = [a.pose];
            struct.pose.position = [b.position];
            struct.pose.orientation = [b.orientation];
            struct.time = [d.time] - t0;
        case 'sensor_msgs/Range'
            struct.range = [a.range];
            struct.time = [d.time] -t0; % This could also use the header time
        case 'relative_nav_msgs/VOUpdate'
            b = [a.transform];
            struct.current_keyframe_id = [a.current_keyframe_id];
            struct.new_keyframe = [a.new_keyframe];
            struct.valid_transformation = [a.valid_transformation];
            struct.transform.translation = [b.translation];
            struct.transform.rotation = [b.rotation];
            [r,p,y] = rollPitchYawFromQuaternion(struct.transform.rotation.');
            struct.transform.euler = [r,p,y]*180/pi;
            struct.time = [d.time] -t0; % This could also use the header time
        case 'relative_nav_msgs/Command'
            struct.commands = [a];
            struct.time = [d.time] -t0; % This could also use the header time
        case 'evart_bridge/transform_plus'
            b = [a.transform];
            struct.transform.translation = [b.translation];
            struct.transform.rotation = [b.rotation];
            struct.transform.euler = rollPitchYawFromQuaternion(struct.transform.rotation.')*180/pi;
            struct.euler = [a.euler];
            struct.velocity = [a.velocity];
            struct.time = [d.time] -t0; % This could also use the header time
        case 'relative_nav_msgs/Edge'
            b = [a.transform];
            struct.transform.translation = [b.translation];
            struct.transform.rotation = [b.rotation];
            struct.transform.euler = rollPitchYawFromQuaternion(struct.transform.rotation.')*180/pi;
            struct.from_node_id = [a.from_node_id];
            struct.to_node_id = [a.to_node_id];
            struct.covariance = [a.covariance];
            struct.time = [d.time] -t0; % This could also use the header time
        case 'nav_msgs/Odometry'
            b = [a.pose];
            c = [b.pose];
            struct.pose.position = [c.position];
            struct.pose.orientation = [c.orientation];
            struct.time = [d.time] -t0; % This could also use the header time
        case 'geometry_msgs/Point'
            struct.point = a;
            struct.time = [d.time] -t0; % This could also use the header time
        case 'fcu_common/RCRaw'
            struct.time = [d.time] -t0;
            struct.values = [a.values];
        case 'ublox_msgs/NavPOSLLH'
            struct.lon = double([a.lon])/1e7;
            struct.lat = double([a.lat])/1e7;
            struct.hAcc = [a.hAcc];
            [struct.x,struct.y,struct.zone] = deg2utm(struct.lat,struct.lon);
        case 'sensor_msgs/NavSatFix'
            struct.lon = [a.longitude];
            struct.lat = [a.latitude];
            cov = [a.position_covariance];
            struct.cov = cov(1,:);
            [struct.x,struct.y,struct.zone] = deg2utm(struct.lat,struct.lon);
            struct.time = [d.time] -t0; % This could also use the header time
        case 'sensor_msgs/Imu'
            %             keyboard;
            struct.acc = [a.linear_acceleration];
            struct.gyro = [a.angular_velocity];
            struct.time = [d.time] -t0; % This could also use the header time
        case 'sensor_msgs/LaserScan'
            struct.angle_min = [a.angle_min];
            struct.angle_max = [a.angle_max];
            struct.angle_increment = [a.angle_increment];
            struct.time_increment = [a.time_increment];
            struct.range_min = [a.range_min];
            struct.range_max = [a.range_max];
            struct.ranges = [a.ranges];
            struct.intensities = [a.intensities];
        case 'fcu_common/Airspeed'
            h = [a.header];
            hs = [h.stamp];
            struct.time = [hs.time] - t0;
            struct.velocity = [a.velocity];
            struct.diff_press = [a.differential_pressure];
            struct.temperature = [a.temperature];
        case 'visualization_msgs/Marker'
            number = 1;
            for i = a(end:-1:1)
                if(size(i.points,2) > 2)
                    if(number == 1)
                        struct.opt.points = i.points;
                        number = 2;
                    elseif(number == 2)
                        struct.unopt.points = i.points;
                        break
                    else
                        break
                    end
                end
            end
        otherwise
            fprintf('     Type: %s not yet supported!\n',type{:});
            continue
    end
    
    % Split topic name into sections
    topic_parts = strsplit(topic{1},'/');
    topic_parts(cellfun('isempty', topic_parts)) = [];
    switch size(topic_parts,2)
        case 1
            data.(topic_parts{1}) = struct;
        case 2
            data.(topic_parts{1}).(topic_parts{2}) = struct;
        case 3
            data.(topic_parts{1}).(topic_parts{2}).(topic_parts{3}) = struct;
        case 4
            data.(topic_parts{1}).(topic_parts{2}).(topic_parts{3}).(topic_parts{4}) = struct;
        case 5
            data.(topic_parts{1}).(topic_parts{2}).(topic_parts{3}).(topic_parts{4}).(topic_parts{5}) = struct;
        otherwise
            fprintf('Too long');
    end
end


end
