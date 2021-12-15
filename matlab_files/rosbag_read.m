%% Load the files
bag = rosbag("Thrust_Exp/2021-12-14-15-02-09-1-0-5.bag");

%TODO: adjust for different experiments

% Commanded Position and Velocity
% %Setpoint by me
% bSel = select(bag,'Topic','mavros/setpoint_raw/local');
% ts_pos_sp = timeseries(bSel,...
%     'Header.Stamp.Sec',...
%     'Header.Stamp.Nsec',...    
%     'Position.Z','Velocity.Z'); 
% % Setpoint by system
% bSel = select(bag,'Topic','mavros/setpoint_raw/target_local');
% ts_pos_sp2 = timeseries(bSel,...
%     'Header.Stamp.Sec',...
%     'Header.Stamp.Nsec',...
%     'Velocity.Z');
% %    'Position.Z','Velocity.Z');

% Commanded Attitude
%Setpoint by me
% bSel = select(bag,'Topic','mavros/setpoint_raw/attitude');
% ts_att_sp = timeseries(bSel,...
%     'Header.Stamp.Sec',...
%     'Header.Stamp.Nsec',...
%     'Orientation.X','Orientation.Y','Orientation.Z','Orientation.W');
%Setpoint by system
bSel = select(bag,'Topic','mavros/setpoint_raw/target_attitude');
ts_att_sp1 = timeseries(bSel,...
    'Header.Stamp.Sec',...
    'Header.Stamp.Nsec',...
    'Orientation.X','Orientation.Y','Orientation.Z','Orientation.W','Thrust');

%Recorded Position and Attitude
% bSel = select(bag,'Topic','mavros/local_position/pose');
% ts_pos_att = timeseries(bSel,...
%     'Header.Stamp.Sec',...
%     'Header.Stamp.Nsec',...
%     'Pose.Position.X',...
%     'Pose.Position.Y',...
%     'Pose.Position.Z', ...
%     'Pose.Orientation.X',...
%     'Pose.Orientation.Y',...
%     'Pose.Orientation.Z',...
%     'Pose.Orientation.W');

% Recorded Velocity
% bSel = select(bag,'Topic','mavros/local_position/velocity_body'); 
% ts_vel_rate = timeseries(bSel,...
%    'Header.Stamp.Sec',...
%    'Header.Stamp.Nsec',...
%    'Twist.Linear.Z');

%IMU data
bSel = select(bag,'Topic','mavros/imu/data'); 
ts_imu= timeseries(bSel,...
   'Header.Stamp.Sec',...
   'Header.Stamp.Nsec',...
   'LinearAcceleration.X',...
   'LinearAcceleration.Y',...
   'LinearAcceleration.Z');
%% Extract the time series and match
%t_1 = ts_att_sp.Data(:,1)+ts_att_sp.Data(:,2)*10^(-9);
t_att_sp = ts_att_sp1.Data(:,1)+ts_att_sp1.Data(:,2)*10^(-9); %%TODO: t_2 is slightly off but in principle they matcht %round(t_2, 3)
%t_att_sys = ts_pos_att.Data(:,1)+ts_pos_att.Data(:,2)*10^(-9);
%t_test = ts_vel_rate.Data(:,1)+ts_vel_rate.Data(:,2)*10^(-9);
t_imu = ts_imu.Data(2:end,1)+ts_imu.Data(2:end,2)*10^(-9); 

t_att_sp = t_att_sp - t_att_sp(1);
t_imu = t_imu - t_imu(1);
%% Roll/Pitch: Extract Euler Angles from Quaternions and times

att_sp = q2e(ts_att_sp.Data(:,3:6));
%t_att_sp = ts_att_sp.Time - ts_att_sp.Time(1);
att_sys = q2e(ts_pos_att.Data(:,6:9));
%t_att_sys = ts_pos_att.Time - ts_att_sp.Time(1);

% TODO: This for now ignores the time delay between both messages

%% VelZ: Extratct velocity from Time series
vel_sp = ts_pos_sp.Data;
t_vel_sp = ts_pos_sp.Time - ts_pos_sp.Time(1);
vel_sys = ts_vel_rate.Data;
t_vel_sys = ts_vel_rate.Time- ts_pos_sp.Time(1);

%% Thrust and Acc
thrust = ts_att_sp1.Data(:,7);
thrust_interp = interp1(t_att_sp, thrust, t_imu);
imu_data = ts_imu.Data(2:end,3:5);
acc = vecnorm(imu_data.').'-9.81;

figure(1)
plot(t_imu, acc)
hold on 
plot(t_imu,thrust_interp)

%% Analyze system response
ang_id = 1; % 1 for pitch, 2 for roll
figure(1)
hold on 
plot(t_att_sp, att_sp(:,ang_id));
plot(t_att_sys, att_sys(:,ang_id));
% stairs(t_vel_sp, vel_sp);
% stairs(t_vel_sys, vel_sys);

%% Resampling

% att_sys_resample = resample(y, t_att_sp);
%y2 = resample(y2, t_att_sys)
% Plots
% 
% figure(2)
% hold on 
% plot(t_att_sp, att_sp(:,1),'*-');
% plot(t_att_sp, att_sys_resample, 'o-');
% plot(t_att_sys, att_sys(:,1),'o-');
% grid on
% legend("Setpoint", "Resample", "System Response")

%% Cut the Data

Ts = 1/50;
t_start = 27;
t_end = 40;
% pitch_sp = att_sp(t_start/Ts:t_end/Ts,ang_id);
% pitch_sys = att_sys(t_start/Ts:t_end/Ts,ang_id);
% t = t_att_sp(t_start/Ts:t_end/Ts)-t_att_sp(t_start/Ts);
% vel_sp = vel_sp(t_start/Ts:t_end/Ts,ang_id);
% vel_sys = vel_sys(t_start/Ts:t_end/Ts,ang_id);
% t = t_vel_sp(t_start/Ts:t_end/Ts)-t_vel_sp(t_start/Ts);
t = t_imu(t_start/Ts:t_end/Ts) - t_imu(t_start/Ts);
thrust = thrust_interp(t_start/Ts:t_end/Ts);
acc = acc(t_start/Ts:t_end/Ts);

%% Show data after cutting it 

figure(2)
hold on 
% plot(t, pitch_sp)
% plot(t, pitch_sys)
% plot(t, vel_sp)
% plot(t, vel_sys)
plot(t,thrust)
plot(t, acc)
%% Save Inputs and Outputs

save("2021-12-14-15-02-09-1-0-5.mat",'t','thrust','acc');

%% Functions

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