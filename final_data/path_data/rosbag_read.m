% With this file you can read all your rosbag files and convert them to a
% .mat file. 
clc;
clear;
%% Load the file path and settings
% Select the correct file path
bag = rosbag("lmpcc_simplesim_tracking_xy_field_2023-01-20-11-50-05.bag")
path = extractBefore(bag.FilePath,".bag");

%% Read bag file
bSel = select(bag,'Topic','drone_hovergames/state');
ts_state= timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Twist.Twist.Linear.X',...
           'Twist.Twist.Linear.Y',...
           'Pose.Pose.Position.X',...
           'Pose.Pose.Position.Y');

%% Extract relevant data
x_state = ts_state.Data(:,5);
y_state = ts_state.Data(:,6);

% Check data
figure(1)
plot(x_state, y_state)

%% Save data
path_data = [x_state, y_state];
csvwrite(strcat(path,'.csv'),path_data)

