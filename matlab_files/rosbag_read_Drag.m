clc 
clear all

%% Load the files 
bag = rosbag("Weird_Behavior/2021-12-22-12-17-39-4-0-1.bag");
% Extract path 
path = extractBefore(bag.FilePath,".bag");
exp_ = str2double(path(end));

%% Load data
% Attitude
bSel = select(bag,'Topic','mavros/local_position/pose');
ts_att = timeseries(bSel,...
    'Header.Stamp.Sec',...
    'Header.Stamp.Nsec',...
    'Pose.Position.Y',...
    'Pose.Orientation.X', ...
    'Pose.Orientation.Y', ...
    'Pose.Orientation.Z', ...
    'Pose.Orientation.W');
% Velocity
bSel = select(bag,'Topic','mavros/local_position/velocity_body'); 
ts_vel = timeseries(bSel,...
   'Header.Stamp.Sec',...
   'Header.Stamp.Nsec',...
   'Twist.Linear.X',...
   'Twist.Linear.Y',...
   'Twist.Linear.Z');
% Acceleration
bSel = select(bag,'Topic','mavros/imu/data');
ts_imu = timeseries(bSel,...
   'Header.Stamp.Sec',...
   'Header.Stamp.Nsec',...
   'Orientation.X',...
   'Orientation.Y',...
   'Orientation.Z',...
   'Orientation.W',...
   'LinearAcceleration.X',...
   'LinearAcceleration.Y',...
   'LinearAcceleration.Z');
bSel = select(bag,'Topic','mavros/imu/data_raw');
ts_imu_raw = timeseries(bSel,...
   'Header.Stamp.Sec',...
   'Header.Stamp.Nsec',...
   'Orientation.X',...
   'Orientation.Y',...
   'Orientation.Z',...
   'Orientation.W',...
   'LinearAcceleration.X',...
   'LinearAcceleration.Y',...
   'LinearAcceleration.Z');
%%
% 
% y_pos = ts_att.Data(:,3)
% %plot(y_pos)
% figure()
% hold on
% plot(diff(y_pos)/0.02)
% plot(ts_vel.data(:,4))
% hold on
% plot(diff(diff(y_pos))/(0.02)^2)
% %plot(ts_imu.Data(:,9)-9.81)
% %plot(ts_imu_raw.Data(:,9)-9.81)
% plot(ts_imu.Data(:,8))
% legend('Diff-Pos','Vel','Diff-Vel','Acc')
%% Time stamps
t_att = ts_att.Data(:,1)+ts_att.Data(:,2)*10^(-9);
t_att = t_att - t_att(1);
t_vel = ts_vel.Data(:,1)+ts_vel.Data(:,2)*10^(-9);
t_vel = t_vel - t_vel(1);
t_imu = ts_imu.Data(1:end,1)+ts_imu.Data(1:end,2)*10^(-9);
t_imu = t_imu - t_imu(1);

%% Analyze data to use

% Attention!: IMU data uses FRD convention while EKF states use FLU but it
% seems that the IMU data is already converted to the local frame (not body)

% Attitude conversion
euler_att = q2e(ts_att.Data(1:end,3:6));

% Data
switch exp_
    case 0
        data_att = euler_att(:,1);
        data_vel = ts_vel.Data(1:end,4);
        data_imu = ts_imu.Data(1:end,4);
    case 1
        data_att = euler_att(:,2);
        data_vel = ts_vel.Data(1:end,3);
        data_imu = ts_imu.Data(1:end,3);
end

figure(1)
plot(t_imu, data_att)
hold on
plot(t_imu, data_vel)
plot(t_imu, data_imu)

[x,y] = ginput;
t_start = round(x(1));
t_end = round(x(2));

%% Cut the data accordingly

Ts = round(mean(t_imu(2:end)-t_imu(1:end-1)),2);
[t_imu, data_imu] = cutdata(t_imu, data_imu, t_start, t_end, Ts);
[t_att, data_att] = cutdata(t_att, data_att, t_start, t_end, Ts);
[t_vel, data_vel] = cutdata(t_vel, data_vel, t_start, t_end, Ts);

%% plot again to confirm
figure(2)
plot(t_imu, data_att)
hold on
plot(t_imu, data_vel)
plot(t_imu, -data_imu)

%% Save the experiment data
save(path,'t_imu', 'data_imu', 'data_vel', 'data_att');

%%

function euler = q2e(quatseries)
    q = [quatseries(:,4), quatseries(:,1:3)];
    euler= quat2eul(q,"XYZ");
    %euler = euler*180/pi;
end

function [time, data]= cutdata(time, data, t_start, t_end, Ts)
    time = time(t_start/Ts:t_end/Ts) - time(t_start/Ts);
    data = data(t_start/Ts:t_end/Ts);
end