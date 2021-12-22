clc;
clear all;

%% Specify File Paths

file_name = '2021-12-16-16-57-04-1-1-0'
file_path =strcat('Drag_Exp/',file_name,'.mat');
image_path =strcat('Images/','Drag_Exp/',file_name,'.png');

%% Load and visualize the data

load(file_path)

figure(1)
pos_fig = [0 0 1920 1080];
plot(t_imu, data_att)
hold on
plot(t_imu, data_vel)
plot(t_imu, data_imu)
legend('att','vel','acc')
saveas(gcf,image_path)

%%
g = 9.81;
figure(2)
plot(t_imu, -data_imu)
hold on
plot(t_imu, -data_att*g-1.245*data_vel);

%% Determine drag coefficient
m = 2.0;
kd_ = -(abs(data_imu) - abs(data_att)*g)./(abs(data_vel));
kd = kd_*m;

figure(3)
plot(t_imu, kd)
%ylim([-1,1])

mean(kd)
median(kd)

%% Use linear grey box identification

Ts = round(mean(t_imu(2:end)-t_imu(1:end-1)),2);
par = 0.2;
aux = 9.81;
m = idgrey('lin_drag',par,'c',aux,0);
Ts = round(mean(t_imu(2:end)-t_imu(1:end-1)),2);
data = iddata(data_vel,data_att,Ts);
m_lin = greyest(data,m);
m_lin = ss(-0.2,-9.81,1,0)
figure(4)
compare(data,m_lin);

%% Use nonlinear grey box identification

Order = [1,1,1];
par = -m_lin.A;
m = idnlgrey('nl_drag',Order,par);
m_nonlin = nlgreyest(data,m)
figure(5)
compare(data,m_nonlin);