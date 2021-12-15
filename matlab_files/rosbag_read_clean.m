clc 
clear all

%% Load the files 
bag = rosbag("Yaw_Exp/2021-12-15-17-46-46-1-0-2.bag");

% Extract experiment number
path = extractBefore(bag.FilePath,".bag");
exp_ = str2double(path(end));

% Depending on experiment load different files
switch exp_
    case 2
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
end

%% Extract time stamp
t_sp = readtime(ts_sp);
t_meas = readtime(ts_meas);

% Extract the data 
switch exp_
    case 2
        euler_sp = q2e(ts_sp.Data(:,3:6));
        data_sp = euler_sp(:,3);
        euler_meas = q2e(ts_meas.Data(:,3:6));
        data_meas = euler_meas(:,3);
    case 3
        data_sp = ts_sp.Data(:,4);
        data_meas = ts_meas.Data(:,4);
end

%%  Plot the data

% TODO: select start and end x-pos(time that has to be extracted) and press
% ENTER when done
figure(1)
hold on
plot(t_meas, data_meas)
plot(t_sp, data_sp)

[x,y] = ginput;
t_start = round(x(1));
t_end = round(x(2));

%% Cut the data accordingly
Ts = round(mean(t_sp(2:end)-t_sp(1:end-1)),2);
[t_sp, data_sp] = cutdata(t_sp, data_sp, t_start, t_end, Ts);
[t_meas, data_meas] = cutdata(t_meas, data_meas, t_start, t_end, Ts);

%% plot again to confirm
figure(2)
hold on 
plot(t_sp, data_sp)
plot(t_meas, data_meas)

%% Save the experiment data

save(path,'t_sp', 't_meas', 'data_sp', 'data_meas')

%% Functions
function time = readtime(timeseries)
    time = timeseries.Data(:,1)+timeseries.Data(:,2)*10^(-9);
    time = time - time(1);
end

function [time, data]= cutdata(time, data, t_start, t_end, Ts)
    time = time(t_start/Ts:t_end/Ts) - time(t_start/Ts);
    data = data(t_start/Ts:t_end/Ts);
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