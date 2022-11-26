% With this file you can read all your rosbag files and convert them to a
% .mat file. 
clc;
clear;
%% Load the file path and settings
% Select the correct file path
bag = rosbag("lmpcc_windsimplesim_nominal_model_plus_wind_2022-11-07-10-41-43.bag")
path = extractBefore(bag.FilePath,".bag");
% Choose sampling time
T_s = 0.2;
%%
bSel = select(bag,'Topic','lmpcc/state');
ts_state= timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Twist.Twist.Linear.X',...
           'Twist.Twist.Linear.Y',...
           'Pose.Pose.Position.X',...
           'Pose.Pose.Position.Y');

bSel = select(bag,'Topic','lmpcc/state_prediction');
ts_state_pred = timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Twist.Twist.Linear.X',...
           'Twist.Twist.Linear.Y',...
           'Pose.Pose.Position.X',...
           'Pose.Pose.Position.Y');

bSel = select(bag,'Topic','drone_hovergames/wind');
ts_wind = timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Vector.X',...
           'Vector.Y');

%% Extract time & data

%Extract time
t_state = readtime(ts_state);
t_state_pred = readtime(ts_state_pred);
t_wind = readtime(ts_wind);

%Extract current and predicted velocities 
x_state = ts_state.Data(:,5);
y_state = ts_state.Data(:,6);
vx_state = ts_state.Data(:,3);
vx_state_pred = ts_state_pred.Data(:,3);
vy_state = ts_state.Data(:,4);
vy_state_pred = ts_state_pred.Data(:,4);

%% Load the file path and settings
% Select the correct file path
bag = rosbag("lmpcc_windsimplesim_GP_model_plus_wind_2022-11-07-10-34-29.bag")
path = extractBefore(bag.FilePath,".bag");
% Choose sampling time
T_s = 0.2;
%%
bSel = select(bag,'Topic','lmpcc/state');
ts_state= timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Twist.Twist.Linear.X',...
           'Twist.Twist.Linear.Y',...
           'Pose.Pose.Position.X',...
           'Pose.Pose.Position.Y');

bSel = select(bag,'Topic','lmpcc/state_prediction');
ts_state_pred = timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Twist.Twist.Linear.X',...
           'Twist.Twist.Linear.Y',...
           'Pose.Pose.Position.X',...
           'Pose.Pose.Position.Y');

bSel = select(bag,'Topic','drone_hovergames/wind');
ts_wind = timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Vector.X',...
           'Vector.Y');

%% Extract time & data

%Extract time
t_state = readtime(ts_state);
t_state_pred = readtime(ts_state_pred);
t_wind = readtime(ts_wind);

%Extract current and predicted velocities 
x1_state = ts_state.Data(:,5);
y1_state = ts_state.Data(:,6);
vx_state = ts_state.Data(:,3);
vx_state_pred = ts_state_pred.Data(:,3);
vy_state = ts_state.Data(:,4);
vy_state_pred = ts_state_pred.Data(:,4);

%% Lemniscate 

x =   [9.5,8.2058,5.6167,3.2604,1.4203,-0.13876,-1.7263,-3.6488,-6.0978,-8.6015,-9.4516,-7.7677,-5.1475,-2.8903,-1.1237,0.41722,2.0436,4.0563,6.5852,8.9405, 9.5,8.2058]
y =  [0,2.5813,3.3543,2.6709,1.3606,-0.1387,-1.6221,-2.8618,-3.3499,-2.2245,0.55173,2.8697,3.3098,2.4605,1.0935,-0.41562,-1.8761,-3.0291,-3.2903,-1.8041, 0, 2.5813]

%%

figure(9)
plot(x_state, y_state)
hold on 
plot(x1_state, y1_state)
plot(x,y)
%% 
figure(10)
plot((vx_state(2:end) - vx_state_pred(1:end-1))./0.05)
hold on 
plot(x_state)
% plot(t_state_pred, vx_state_pred)
%%

%Extract the wind 
wind_x = ts_wind.Data(1:end,3);
wind_y = ts_wind.Data(1:end,4);

%% Detect jumps in the state caused by reset
% outlier = isoutlier(x_state(2:end)-x_state(1:end-1));
% trim_outliers = [outlier;1];
% x_state = x_state(~trim_outliers);
% y_state = y_state(~trim_outliers);
% vx_state = vx_state(~trim_outliers);
% vy_state = vy_state(~trim_outliers);
% vx_state_pred = vx_state_pred(~trim_outliers);
% vy_state_pred = vy_state_pred(~trim_outliers);
% t_state_pred = t_state_pred(~trim_outliers);
% t_state = t_state(~trim_outliers);
% wind_x = wind_x(~trim_outliers);
% wind_y = wind_y(~trim_outliers);
%% Plot the velocities in x and y direction 

figure(1)
plot(t_state,vx_state,'-')
hold on
plot(t_state_pred,vx_state_pred,'-')
title('Current versus predicted velocity in x-direction')
legend('state', 'predicted state')

figure(2)
plot(t_state,vy_state,'-')
hold on
plot(t_state_pred,vy_state_pred,'-')
title('Current versus predicted velocity in y-direction')
legend('state', 'predicted state')

%% Plot the path 

figure(3)
plot(x_state, y_state)
title('Quadrotor path')

%% Interpolate the data

t_state = t_state - t_state(1);
t_state_pred = t_state_pred - t_state_pred(1);
t = 0:T_s:t_state(end);
vx_state_interp = interp1(t_state, vx_state, t);
vx_state_pred_interp = interp1(t_state, vx_state_pred, t);
vy_state_interp = interp1(t_state, vy_state, t);
vy_state_pred_interp = interp1(t_state, vy_state_pred, t);

%% Check that the interpolation worked
figure(4)
plot(t, vx_state_interp, '*-')
hold on 
plot(t_state, vx_state, '*-')

figure(5)
plot(t, vx_state_pred_interp, '*-')
hold on 
plot(t_state, vx_state_pred, '*-')

figure(6)
plot(t, vy_state_interp, '*-')
hold on 
plot(t_state, vy_state, '*-')

figure(7)
plot(t, vy_state_pred_interp, '*-')
hold on 
plot(t_state, vy_state_pred, '*-')

%% Compute the difference

%T_s = mean(t_state(2:end)-t_state(1:end-1)); // TODO: talk to Dennis about
%this
T_s = 0.05;
vx_diff = vx_state(7:end) - vx_state_pred(1:end-1);
ax_dist = vx_diff./T_s;
vy_diff = vy_state(7:end) - vy_state_pred(1:end-1);
ay_dist = vy_diff./T_s;

figure(8)
plot(t_state(7:end), ax_dist)
hold on 
plot(t_state(7:end), ay_dist)

%% Plot the training data 

figure(9)
plot3(x_state(2:end),y_state(2:end), ax_dist)
title('Training data in x-direction')
xlabel('Position in x')
ylabel('Position in y')
zlabel('Disturbance in vx')
% saveas(gcf,'training_data.png')

figure(10)
plot3(x_state(2:end),y_state(2:end), ay_dist)
title('Training data in y-direction')
xlabel('Position in x')
ylabel('Position in y')
zlabel('Disturbance in vy')
% saveas(gcf,'training_data.png')

%% Turn it into a .csv file
training_input_x = x_state(7:end);
training_input_y = reshape(y_state(7:end), size(training_input_x));
training_target_vx = reshape(ax_dist, size(training_input_x));
training_target_vy = reshape(ay_dist, size(training_input_x));

training_data = [training_input_x, training_input_y, training_target_vx, training_target_vy, wind_x(6:end), wind_y(6:end)];
csvwrite(strcat(path,'.csv'),training_data)

%% Functions

function [time_sp] = readtime(timeseries_sp)
    time_sp = timeseries_sp.Data(:,1)+timeseries_sp.Data(:,2)*10^(-9);
end