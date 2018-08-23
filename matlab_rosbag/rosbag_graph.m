%% Get info from pixhawk
clear rosbag_wrapper;
clear ros.bag;
clear

%% Load the previous session
load('flight_8_18_18.mat');

%% Get info from rosbag
bagfile_h = '/home/dallin/flight_rosbags/milestone_3/flight_8_18_18/handoff/handoff.bag';
bagfile_t = '/home/dallin/flight_rosbags/milestone_3/flight_8_18_18/tracker/tracker.bag';
addpath('/home/dallin/flight_rosbags/milestone_3/flight_8_18_18/ros_scripts/matlab_rosbag/plot_goole_map');

% topic_names = {'/state' '/gimbal/control'};
% topics = cellstr(topic_names);
% msgs1 = processTopics(topics,bagfile1);
[msgs_h, errors_h, warnings_h, infos_h, error_time_h, warning_time_h, info_time_h] = processAllTopics(bagfile_h);
[msgs_t, errors_t, warnings_t, infos_t, error_time_t, warning_time_t, info_time_t] = processAllTopics(bagfile_t);

%% Plot Ground target trajectory
gt_pos_global = [msgs_h.ground_target.mavros.global_position.global.latitude; 
                 msgs_h.ground_target.mavros.global_position.global.longitude;
                 msgs_h.ground_target.mavros.global_position.global.altitude - ...
                 msgs_h.ground_target.mavros.global_position.global.altitude(1)]';
             
hmav_pos_global = [msgs_h.handoff.mavros.global_position.global.latitude;
                 msgs_h.handoff.mavros.global_position.global.longitude;
                 msgs_h.handoff.mavros.global_position.global.altitude - ...
                 msgs_h.handoff.mavros.global_position.global.altitude(1)]';
             
hmav_waypoints = [[msgs_h.handoff.mavros.mission.waypoints.waypoints.x_lat]; 
                  [msgs_h.handoff.mavros.mission.waypoints.waypoints.y_long]; 
                  [msgs_h.handoff.mavros.mission.waypoints.waypoints.z_alt]]';
              
tmav_pos_global = [msgs_t.tracker.mavros.global_position.global.latitude;
                 msgs_t.tracker.mavros.global_position.global.longitude;
                 msgs_t.tracker.mavros.global_position.global.altitude - ...
                 msgs_t.tracker.mavros.global_position.global.altitude(1)]';
             
tmav_waypoints = [[msgs_t.tracker.mavros.mission.waypoints.waypoints.x_lat];
                 [msgs_t.tracker.mavros.mission.waypoints.waypoints.y_long];
                 [msgs_t.tracker.mavros.mission.waypoints.waypoints.z_alt]]';
             
t_modes = msgs_t.tracker.mavros.state.mode;
             
waypoints_anim = animatedline('Color', 'y', 'MaximumNumPoints', 1, 'markers', 12);
waypoints_anim.LineStyle = ':';

t_animation = animatedline('Color', 'y', 'MaximumNumPoints', 250, 'LineWidth', 3);

set(gca, 'XLim', [-111.9074 -111.8955], 'YLim', [40.3606 40.3660]);
plot_google_map('Maptype', 'satellite')

for i=1:length(tmav_pos_global(:,1)-20)
    t_mode_current = t_modes(int16(i/(length(tmav_pos_global(:,1))/length(t_modes)))+1);
    if strcmp(t_mode_current, 'GUIDED')
        t_animation.Color = 'b';
    elseif strcmp(t_mode_current, 'AUTO')
        t_animation.Color = 'g';
    elseif strcmp(t_mode_current, 'RTL')
        t_animation.Color = 'm';
    else
        t_animation.Color = 'r';
    end
    addpoints(t_animation, tmav_pos_global(i,2), tmav_pos_global(i,1));
    drawnow;
%     pause(.01);
end

% plot(gt_pos_global(2,:), gt_pos_global(1,:),'r') 
% plot(hmav_pos_global(:,2), hmav_pos_global(:,1), 'b')
% plot(hmav_waypoints(:,2), hmav_waypoints(:,1), '.g', 'markers', 12)
% plot(tmav_pos_global(:,2), tmav_pos_global(:,1), 'y')
% plot(tmav_waypoints(:,2), tmav_waypoints(:,1), '.m', 'markers', 12)
% legend ('Ground target', 'Handoff path', 'Handoff waypoints')
% hold off



%% Plot airspeed
clf;
close all;
figure(1)
subplot(3,1,1);
hold on
plot(msgs_h.state.time,msgs_h.state.Va);
plot(msgs_h.controller_commands.time, msgs_h.controller_commands.Va_c);
legend('Measured', 'Commanded');
hold off;
title('airspeed')
subplot(3,1,2);
hold on
plot(msgs_h.state.time, -msgs_h.state.position(3,:));
plot(msgs_h.controller_commands.time, msgs_h.controller_commands.h_c);
legend('Altitude', 'Commanded');
hold off;
title('Altitude')
subplot(3,1,3)
plot(msgs_h.airspeed.time, msgs_h.airspeed.differential_pressure)
title('Diff press')

%% plot IMU
clf
close all
figure(1)
plot(msgs_h.attitude.time, msgs_h.attitude.attitude);

%% Plot only altitude
% clf;
% close all;
figure(4)
plot(msgs_h.state.time,-msgs_h.state.position(3,:));
title('Altitude')
xlabel('Time (sec)')
ylabel('Height (meters)')

%% Plot gps
clf;
close all;
figure(1)
subplot(2,1,1)
plot(msgs_h.gps.data.time,msgs_h.gps.data.fix)
title('fix status')

figure(2)
gps = [msgs_h.gps.data.latitude;
    msgs_h.gps.data.longitude;
    msgs_h.gps.data.altitude];
inertial = gps - gps(:,1);
plot3(inertial(1,:),inertial(2,:),inertial(3,:));
title('inertial position in 3d')

%% Plot RC signals
% clf;
% close all
figure(3)
hold on;
% plot(msgs.rc_raw.time,msgs.rc_raw.values(1,:));
% plot(msgs.rc_raw.time,msgs.rc_raw.values(2,:));
% plot(msgs.rc_raw.time,msgs.rc_raw.values(3,:));
% plot(msgs.rc_raw.time,msgs.rc_raw.values(4,:));
% plot(msgs.rc_raw.time,msgs.rc_raw.values(5,:));
plot(msgs_h.rc_raw.time,msgs_h.rc_raw.values(6,:));
% plot(msgs.rc_raw.time,msgs.rc_raw.values(7,:));
% plot(msgs.rc_raw.time,msgs.rc_raw.values(8,:));
legend('RC1 Ail', 'RC2 Ele', 'RC3 Thro', 'RC4 Rud', 'RC5', 'RC6');
hold off;

%% Plot Status
clf;
close all
figure(4)
subplot(2,2,1)
plot(msgs_h.status.time,msgs_h.status.armed)
title('Armed state')
subplot(2,2,2)
plot(msgs_h.status.time,msgs_h.status.failsafe)
title('failsafe')
subplot(2,2,3)
plot(msgs_h.status.time,msgs_h.status.rc_override)
title('rc_override')
subplot(2,2,4)
plot(msgs_h.status.time,msgs_h.status.num_errors)
title('Num errors')

%% Plot the position in 3D
clf;
close all;
figure(1)
hold on
plot3(msgs_h.state.position(1,:),msgs_h.state.position(2,:),-msgs_h.state.position(3,:))
% axis([-200 300 -300 50 -10 100])
hold off

%% Plot course
% clf;
% close all
figure(1);
hold on
plot(msgs_h.state.time,msgs_h.state.chi);
plot(msgs_h.controller_commands.time, msgs_h.controller_commands.chi_c);
legend('Chi', 'Commanded Chi');
% axis([0 200 -20 20]);

%% Plot roll
figure(2)
hold on
plot(msgs_h.state.time, msgs_h.state.phi)
plot(msgs_h.controller_inners.time, msgs_h.controller_inners.phi_c);
plot(msgs_h.rc_raw.time,(msgs_h.rc_raw.values(6,:)-1500)/1000);
legend('Measured', 'Commanded', 'RC signal');

%% Plot the baro
% clf;
figure(2)
% subplot(2,2,1)
hold on
plot(msgs_h.baro.time,msgs_h.baro.pressure/1000)
% plot(msgs.ins.baro.time,msgs.ins.baro.fluid_pressure)
% plot((BARO(2970:end,2)-BARO(2970,2))/10e5,BARO(2970:end,4)/1000)
legend('Rosflight', 'INS', 'Pixhawk');
ylabel('kPa')
xlabel('Time (sec)')
hold off

figure(3)
hold on
plot(msgs_h.baro.time,msgs_h.baro.altitude/1000)
% plot(msgs.ins.baro.time,msgs.ins.

hold off


%%
clf;
close all;
figure(1)
ell = zeros(3,length(msgs_h.gimbal.control.vector));
gimbal = msgs_h.gimbal.control.vector;
gimbal_position = [];
phi = [];
theta = [];
psi = [];
gimbal_point = [];
gimbal_plot = [];

ell(3,:) = sin(gimbal(3,:));
for i = 1:5:length(msgs_h.state.time)
    gimbal_position(:,end+1) = msgs_h.state.position(:,i);
    phi(end+1) = msgs_h.state.phi(i);
    theta(end+1) = msgs_h.state.theta(i);
    psi(end+1) = msgs_h.state.psi(i);
    
end


for i = 1:length(gimbal)
    
cphi = cos(phi(i));
sphi = sin(phi(i));
ctheta = cos(theta(i));
stheta = sin(theta(i));
cpsi = cos(psi(i));
spsi = sin(psi(i));
cel = cos(gimbal(2,i));
sel = sin(gimbal(2,i));
caz = cos(gimbal(3,i));
saz = sin(gimbal(3,i));


R_v_to_b = [ctheta*cpsi, ctheta*spsi, -stheta;
    sphi*stheta*cpsi-cphi*spsi, sphi*stheta*spsi+cphi*cpsi, sphi*ctheta;
    cphi*stheta*cpsi+sphi*spsi, cphi*stheta*spsi-sphi*cpsi, cphi*ctheta];

R_b_to_g = [cel*caz, cel*saz, -sel;
    -saz, caz, 0;
    sel*caz, sel*saz, cel];
rot = R_v_to_b'*R_b_to_g';
gimbal_point(:,i) = rot*(gimbal_position(:,i)+20);
end

for i = 1:20:length(gimbal_point)
    gimbal_plot(:,end+1) = gimbal_position(:,i);
    gimbal_plot(:,end+1) = gimbal_point(:,i);
    gimbal_plot(:,end+1) = gimbal_position(:,i);
end


hold on
plot3(msgs_h.state.position(1,:),msgs_h.state.position(2,:),-msgs_h.state.position(3,:))
plot3(0,0,0,'*')
plot3(gimbal_plot(1,:),gimbal_plot(2,:),-gimbal_plot(3,:))
xlabel('Position N')
ylabel('Position E')
zlabel('Position -D')
hold off
% 
% figure(2)
% hold on
% plot(msgs5.state.time,msgs5.state.psi)
% plot(msgs5.state.time,msgs5.state.chi)
% plot(msgs5.gimbal.control.time, msgs5.gimbal.control.vector(3,:))
% legend('Psi', 'Chi', 'Gimbal az')







