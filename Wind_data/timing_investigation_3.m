bag = rosbag("simplecontrol_windsimplesim_test_timing_2022-10-20-14-03-43.bag")
path = extractBefore(bag.FilePath,".bag");

%% Read Odometry data
bSel = select(bag,'Topic','/mavros/local_position/odom');
ts_mavros_state = timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Pose.Pose.Position.X',...
           'Pose.Pose.Position.Y',...
           'Twist.Twist.Linear.X',...
           'Twist.Twist.Linear.Y',...
           'Pose.Pose.Orientation.X',...
           'Pose.Pose.Orientation.Y',...
           'Pose.Pose.Orientation.Z',...
           'Pose.Pose.Orientation.W');

%% Read Input data
bSel = select(bag,'Topic','/mavros/setpoint_raw/target_attitude');
ts_mavros_attitude = timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Orientation.X',...
           'Orientation.Y',...
           'Orientation.Z',...
           'Orientation.W', ...
           'Thrust');

%% Read state prediction data
bSel = select(bag,'Topic','drone_hovergames/state_prediction');
ts_hovergames_state_pred = timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Pose.Pose.Position.X',...
           'Pose.Pose.Position.Y',...
           'Twist.Twist.Linear.X',...
           'Twist.Twist.Linear.Y',...
           'Pose.Pose.Orientation.X',...
           'Pose.Pose.Orientation.Y',...
           'Pose.Pose.Orientation.Z',...
           'Pose.Pose.Orientation.W');

%% Read the wind data
bSel = select(bag,'Topic','drone_hovergames/wind');
ts_hovergames_wind_pred = timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Accel.Linear.X',...
           'Accel.Linear.Y',...
           'Accel.Angular.X',...
           'Accel.Angular.Y',...
           'Accel.Angular.Z');

%% Collect data
[t_state, t_att] = readtime(ts_mavros_state, ts_mavros_attitude);
[t_state_pred, t_wind] = readtime(ts_hovergames_state_pred, ts_mavros_attitude);

% Input command
euler_inp = q2e(ts_mavros_attitude.Data(:,3:6));
thrust_inp = ts_mavros_attitude.Data(:,7);

% Current state
state = ts_mavros_state.Data(:,3:6);
state_eul = q2e(ts_mavros_state.Data(:,7:10));
state= [state, state_eul];

% State prediction
state_pred = ts_hovergames_state_pred.Data(:,3:6);
state_eul_pred = q2e( ts_hovergames_state_pred.Data(:,7:10));
state_pred = [state_pred, state_eul_pred];

% Wind prediction 
% wind_pred = ts_hovergames_wind_pred.Data(:,3:7);

%% Save to .csv
data_inp = [t_att, euler_inp, thrust_inp]
csvwrite("timing_investigation_input.csv", data_inp(52:3752,:))
data_state = [t_state, state]
csvwrite("timing_investigation_state.csv", data_state(29:3729,:))
data_state_pred = [t_state_pred, state_pred]
csvwrite("timing_investigation_state_pred.csv", data_state_pred(:,:))
% data_wind = [t_wind, wind_pred]
% csvwrite("timing_investigation_wind.csv", data_wind(762:1562,:))

%%
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