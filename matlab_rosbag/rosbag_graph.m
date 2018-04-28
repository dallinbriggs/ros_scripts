%% Get info from pixhawk
clear rosbag_wrapper;
clear ros.bag;
clear

%% Get info from rosbag
bagfile = '/media/dallin/32F8BC1FF8BBDEF3/20180201PM/flight2/orbit_test_2018-01-03-18-30-36_9.bag';

% topic_names = {'/state' '/gimbal/control'};
% topics = cellstr(topic_names);
% msgs1 = processTopics(topics,bagfile1);
[msgs, errors, warnings, infos, error_time, warning_time, info_time] = processAllTopics(bagfile);

%% Plot truth and estimate
ins_time = msgs.handoff_mav.ins.imu1.time;
ins_acc = msgs.handoff_mav.ins.imu1.acc;
ins_gyro = msgs.handoff_mav.ins.imu1.gyro;

clf
% close all
figure(1)
hold on
plot(ins_time, ins_acc)
title('INS Acceleration')
xlabel('Time (seconds)')
ylabel('Acceleration (m/s/s)')
legend('X accel', 'Y accel', 'Z accel')
hold off

figure(2)
hold on
plot(ins_time, ins_gyro)
title('INS Gyro')
legend('X Gryo', 'Y Gyro', 'Z Gyro')
hold off



%% Plot airspeed
clf;
close all;
figure(1)
subplot(3,1,1);
hold on
plot(msgs.state.time,msgs.state.Va);
plot(msgs.controller_commands.time, msgs.controller_commands.Va_c);
legend('Measured', 'Commanded');
hold off;
title('airspeed')
subplot(3,1,2);
hold on
plot(msgs.state.time, -msgs.state.position(3,:));
plot(msgs.controller_commands.time, msgs.controller_commands.h_c);
legend('Altitude', 'Commanded');
hold off;
title('Altitude')
subplot(3,1,3)
plot(msgs.airspeed.time, msgs.airspeed.differential_pressure)
title('Diff press')

%% plot IMU
clf
close all
figure(1)
plot(msgs.attitude.time, msgs.attitude.attitude);

%% Plot only altitude
% clf;
% close all;
figure(4)
plot(msgs.state.time,-msgs.state.position(3,:));
title('Altitude')
xlabel('Time (sec)')
ylabel('Height (meters)')

%% Plot gps
clf;
close all;
figure(1)
subplot(2,1,1)
plot(msgs.gps.data.time,msgs.gps.data.fix)
title('fix status')

figure(2)
gps = [msgs.gps.data.latitude;
    msgs.gps.data.longitude;
    msgs.gps.data.altitude];
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
plot(msgs.rc_raw.time,msgs.rc_raw.values(6,:));
% plot(msgs.rc_raw.time,msgs.rc_raw.values(7,:));
% plot(msgs.rc_raw.time,msgs.rc_raw.values(8,:));
legend('RC1 Ail', 'RC2 Ele', 'RC3 Thro', 'RC4 Rud', 'RC5', 'RC6');
hold off;

%% Plot Status
clf;
close all
figure(4)
subplot(2,2,1)
plot(msgs.status.time,msgs.status.armed)
title('Armed state')
subplot(2,2,2)
plot(msgs.status.time,msgs.status.failsafe)
title('failsafe')
subplot(2,2,3)
plot(msgs.status.time,msgs.status.rc_override)
title('rc_override')
subplot(2,2,4)
plot(msgs.status.time,msgs.status.num_errors)
title('Num errors')

%% Plot the position in 3D
clf;
close all;
figure(1)
hold on
plot3(msgs.state.position(1,:),msgs.state.position(2,:),-msgs.state.position(3,:))
% axis([-200 300 -300 50 -10 100])
hold off

%% Plot course
% clf;
% close all
figure(1);
hold on
plot(msgs.state.time,msgs.state.chi);
plot(msgs.controller_commands.time, msgs.controller_commands.chi_c);
legend('Chi', 'Commanded Chi');
% axis([0 200 -20 20]);

%% Plot roll
figure(2)
hold on
plot(msgs.state.time, msgs.state.phi)
plot(msgs.controller_inners.time, msgs.controller_inners.phi_c);
plot(msgs.rc_raw.time,(msgs.rc_raw.values(6,:)-1500)/1000);
legend('Measured', 'Commanded', 'RC signal');

%% Plot the baro
% clf;
figure(2)
% subplot(2,2,1)
hold on
plot(msgs.baro.time,msgs.baro.pressure/1000)
% plot(msgs.ins.baro.time,msgs.ins.baro.fluid_pressure)
% plot((BARO(2970:end,2)-BARO(2970,2))/10e5,BARO(2970:end,4)/1000)
legend('Rosflight', 'INS', 'Pixhawk');
ylabel('kPa')
xlabel('Time (sec)')
hold off

figure(3)
hold on
plot(msgs.baro.time,msgs.baro.altitude/1000)
% plot(msgs.ins.baro.time,msgs.ins.

hold off


%%
clf;
close all;
figure(1)
ell = zeros(3,length(msgs.gimbal.control.vector));
gimbal = msgs.gimbal.control.vector;
gimbal_position = [];
phi = [];
theta = [];
psi = [];
gimbal_point = [];
gimbal_plot = [];

ell(3,:) = sin(gimbal(3,:));
for i = 1:5:length(msgs.state.time)
    gimbal_position(:,end+1) = msgs.state.position(:,i);
    phi(end+1) = msgs.state.phi(i);
    theta(end+1) = msgs.state.theta(i);
    psi(end+1) = msgs.state.psi(i);
    
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
plot3(msgs.state.position(1,:),msgs.state.position(2,:),-msgs.state.position(3,:))
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







