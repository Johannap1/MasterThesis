clc 
clear all

%% Load the files 
bag = rosbag("Roll_Exp/2021-12-22-15-23-03-2-1-0.bag");
% Extract path and experiment type
path = extractBefore(bag.FilePath,".bag");
exp_ = str2double(path(end));
%exp_ = 5; %Override in case of hovering experiment

% Depending on experiment load different files
switch exp_
    case {0,1,2}
        % Load the setpoint
        bSel = select(bag,'Topic','mavros/setpoint_raw/target_attitude');
        ts_sp = timeseries(bSel,...
            'Header.Stamp.Sec',...
            'Header.Stamp.Nsec',...
            'Orientation.X',...
            'Orientation.Y',...
            'Orientation.Z',...
            'Orientation.W');
        %Load the measured data
        bSel = select(bag,'Topic','mavros/local_position/pose');
        ts_meas = timeseries(bSel,...
            'Header.Stamp.Sec',...
            'Header.Stamp.Nsec',...
            'Pose.Orientation.X', ...
            'Pose.Orientation.Y', ...
            'Pose.Orientation.Z', ...
            'Pose.Orientation.W');
    case 3
        % Load the setpoint
        bSel = select(bag,'Topic','mavros/setpoint_raw/target_attitude');
        ts_sp = timeseries(bSel,...
            'Header.Stamp.Sec',...
            'Header.Stamp.Nsec',...
            'BodyRate.X',...
            'BodyRate.Y',...
            'BodyRate.Z');
        % Load the measured data
        bSel = select(bag,'Topic','mavros/local_position/velocity_body'); 
        ts_meas = timeseries(bSel,...
            'Header.Stamp.Sec',...
            'Header.Stamp.Nsec',...
            'Twist.Angular.X',...
            'Twist.Angular.Y',...
            'Twist.Angular.Z');
    case 5
        % Load the setpoint
        bSel = select(bag,'Topic','mavros/setpoint_raw/target_attitude');
        ts_sp = timeseries(bSel,...
            'Header.Stamp.Sec',...
            'Header.Stamp.Nsec',...
            'Thrust');
        % Load the measured data
        bSel = select(bag,'Topic','mavros/imu/data'); 
        ts_meas= timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'LinearAcceleration.Z');
end

%% Extract time & data

% time stamp
% TODO: take care of the fact that the two time stamps do not match
[t_sp, t_meas] = readtime(ts_sp,ts_meas);

% Extract the data 
switch exp_
    case 0
        euler_sp = q2e(ts_sp.Data(:,3:6));
        data_sp = euler_sp(:,1);
        euler_meas = q2e(ts_meas.Data(:,3:6));
        data_meas = euler_meas(:,1);
    case 1
        euler_sp = q2e(ts_sp.Data(:,3:6));
        data_sp = euler_sp(:,2);
        euler_meas = q2e(ts_meas.Data(:,3:6));
        data_meas = euler_meas(:,2);
    case 2
        euler_sp = q2e(ts_sp.Data(:,3:6));
        data_sp = euler_sp(:,3);
        euler_meas = q2e(ts_meas.Data(:,3:6));
        data_meas = euler_meas(:,3);
    case 3
        data_sp = ts_sp.Data(:,4);
        data_meas = ts_meas.Data(:,4);
    case 5
        data_sp = ts_sp.Data(:,3);
        data_meas = ts_meas.Data(:,3);
end

%% Yaw drift investigation 
% 
% figure(1)
% plot(t_sp, euler_sp);
% hold on 
% plot(t_meas, euler_meas(:,3));
% legend('Roll_sp','Pitch_sp','Yaw_sp','Yaw')

%% Interpolate the data and reset time

data_meas_interp = interp1(t_meas, data_meas, t_sp);
t_meas = t_meas -t_meas(1);
t_sp = t_sp - t_sp(1);

%%  Plot the data

% TODO: select start and end x-pos(time that has to be extracted) and press
% ENTER when done
figure(2)
hold on
plot(t_meas, data_meas)
plot(t_sp, data_meas_interp)
plot(t_sp, data_sp)

[x,y] = ginput;
t_start = round(x(1));
t_end = round(x(2));

%% Cut the data accordingly

Ts = round(mean(t_sp(2:end)-t_sp(1:end-1)),2);
[t_sp, data_meas_interp, data_sp] = cutdata(t_meas, data_meas_interp, data_sp, t_start, t_end, Ts);

%% plot again to confirm
figure(3)
hold on 
plot(t_sp, data_sp)
plot(t_sp, data_meas_interp)

%% Save the experiment data
data_meas = data_meas_interp;
save(path,'t_sp','data_sp', 'data_meas')

%% Functions

function [time_sp, time_meas] = readtime(timeseries_sp, timeseries_meas)
    time_sp = timeseries_sp.Data(:,1)+timeseries_sp.Data(:,2)*10^(-9);
    time_meas = timeseries_meas.Data(:,1)+timeseries_meas.Data(:,2)*10^(-9);
end

function [time, data1, data2]= cutdata(time, data1, data2, t_start, t_end, Ts)
    time = time(t_start/Ts:t_end/Ts) - time(t_start/Ts);
    data1 = data1(t_start/Ts:t_end/Ts);
    data2 = data2(t_start/Ts:t_end/Ts);
end

function euler = q2e(quatseries)
    euler= quat2eul(quatseries);
    for i = 1:length(euler)
        if(euler(i,1)<0)
            euler(i,1) = euler(i,1)+2*pi;
        end
        euler(i,1) = euler(i,1)-pi;
    end
    euler = euler*180/pi;
end