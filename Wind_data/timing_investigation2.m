clc;
clear;

%% Load the file path and settings
% Select the correct file path
bag = rosbag("id-thrust-hover_2022-10-11-11-32-16.bag")
path = extractBefore(bag.FilePath,".bag");
% Choose sampling time
T_s = 0.02;
% Chose experiment type
% 0: roll ID experiment 
% 1: pitch ID experiment
% 2: yaw ID experiment
% 3: way rate ID experiment
% 5: thrust sim ID experiment 
% 6: thrust exp ID experiment
exp_ = 0;
% Choose experiment name to be figure title 
exp_name = "ID Roll Experiment - PRBS-1-4-12deg"

%% Load rosbag files and topics
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
            'Orientation.W',...
            'Thrust');
        %Load the measured data
        bSel = select(bag,'Topic','mavros/local_position/odom');
        ts_meas = timeseries(bSel,...
            'Header.Stamp.Sec',...
            'Header.Stamp.Nsec',...
            'Pose.Pose.Orientation.X', ...
            'Pose.Pose.Orientation.Y', ...
            'Pose.Pose.Orientation.Z', ...
            'Pose.Pose.Orientation.W', ...
            'Twist.Twist.Linear.Y');
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
    case 6
        % Load the setpoint
        bSel = select(bag,'Topic','mavros/setpoint_raw/target_attitude');
        ts_sp = timeseries(bSel,...
            'Header.Stamp.Sec',...
            'Header.Stamp.Nsec',...
            'Thrust');
        % Load the measured data
        bSel = select(bag,'Topic','mavros/battery'); 
        ts_meas= timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Voltage');
end

%% Extract time & data

%Extract time
[t_sp, t_meas] = readtime(ts_sp,ts_meas);

% Extract the data 
switch exp_
    case 0
        euler_sp = q2e(ts_sp.Data(:,3: ...
            6));
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
    case 6
        data_sp = ts_sp.Data(:,3);
        data_meas = ts_meas.Data(:,3);
end

% data_meas = ts_meas.Data(:,7)
data_sp = ts_sp.Data(:,7)
%Clean data in case of time jumps or doubly defined time stamps
[t_sp, data_sp] = cleandata(t_sp, data_sp);
[t_meas, data_meas] = cleandata(t_meas, data_meas);

%% Plot the data 

figure(1)
hold on
plot(t_sp(253:1253), data_sp(253:1253),'r')
%plot(t_meas(1459:2459), data_meas(1459:2459),'b')

%%
% data = [t_meas(1458:2459), data_meas(1458:2459)]
% csvwrite("timing_investigation_speed.csv", data)
% data_sp1 = [t_sp(252:1253), data_sp(252:1253)]
% csvwrite("timing_investigation_sp_speed.csv", data_sp1)
data_thrust = [t_sp(252:1253), data_sp(252:1253)]
csvwrite("timing_investigation_thrust.csv", data_thrust)



%% Functions

function [time_sp, time_meas] = readtime(timeseries_sp, timeseries_meas)
    time_sp = timeseries_sp.Data(:,1)+timeseries_sp.Data(:,2)*10^(-9);
    time_meas = timeseries_meas.Data(:,1)+timeseries_meas.Data(:,2)*10^(-9);
end

function [time, data1, data2]= cutdata(time, data1, data2, t_start, t_end, Ts)
    time = time(round(t_start/Ts):round(t_end/Ts)) - time(round(t_start/Ts));
    data1 = data1(round(t_start/Ts):round(t_end/Ts));
    data2 = data2(round(t_start/Ts):round(t_end/Ts));
end

function [t, data] = cleandata(t, data)
    % Remove data where time jump occurred
    clean_data_sp = find(t < 10000);
    t = t(clean_data_sp);
    data = data(clean_data_sp);
    % Remove double defined timestamps
    [t, idx] = unique(t);
    data = data(idx);
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