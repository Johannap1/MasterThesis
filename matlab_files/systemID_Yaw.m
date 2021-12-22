clc;
clear all;

%% Load the data 

file_name = '2021-12-15-17-06-52-0-0-2';
file_path =strcat('Yaw_Exp/',file_name,'.mat');
image_path =strcat('Images/','Yaw_Exp/',file_name,'.png');

load(file_path)

figure(1)
pos_fig = [0 0 1920 1080];
plot(t_sp, data_sp)
hold on 
plot(t_meas, data_meas);
grid on
legend("Setpoint","System Response")
title(file_name)
xlabel("time [s]")
ylabel("amplitude [deg]")
%saveas(gcf,image_path)

%% Store the data in right format

Ts = round(mean(t_meas(2:end)-t_meas(1:end-1)),2);
data = iddata(data_meas,data_sp,Ts);

%% System ID

%n4sid
nx = 2; %model order
sys = tfest(data, nx, 0); %sys = n4sid(data,nx,'DisturbanceModel','none','N4Weight','MOESP');
figure(2)
pos_fig = [0 0 1920 1080];
set(gcf,'Position',pos_fig)
compare(data,sys)
image_path =strcat('Images/','Yaw_Exp/',file_name,'-ID.png');
saveas(gcf,image_path)

%% Load some verification data 

file_name = '2021-12-15-17-46-46-1-0-2';
file_path =strcat('Yaw_Exp/',file_name,'.mat');
image_path =strcat('Images/','Yaw_Exp/',file_name,'.png');

load(file_path)

figure(3)
pos_fig = [0 0 1920 1080];
plot(t_sp, data_sp)
hold on 
plot(t_meas, data_meas);
grid on
legend("Setpoint","System Response")
title(file_name)
xlabel("time [s]")
ylabel("amplitude [deg]")
saveas(gcf,image_path)

ver_data1 = iddata(data_meas,data_sp,Ts);

%% Compare

figure(4)
pos_fig = [0 0 1920 1080];
set(gcf,'Position',pos_fig)
compare(ver_data1,sys)
image_path =strcat('Images/',file_name,'-ID.png');