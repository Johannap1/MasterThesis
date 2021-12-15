clc;
clear all;

%% Load the data 

file_name = '2021-12-07-15-38-44-1-1-4';
file_path =strcat(file_name,'.mat');
image_path =strcat('Images/',file_name,'.png');

%%
load(file_path)

figure(1)
pos_fig = [0 0 1920 1080];
set(gcf,'Position',pos_fig)
hold on 
plot(t, vel_sp);
plot(t, vel_sys);
grid on
legend("Setpoint","System Response")
title(file_name)
xlabel("time [s]")
ylabel("velocity [m/s]")
saveas(gcf,image_path)

%% Store the data in right format

Ts = 0.02;
data = iddata(vel_sys,vel_sp,Ts);

%% Estimate the time delay

% TODO
% nk = delayest(data);
%Shift the data accordingly
% y = pitch_sys(1+nk:end);
% u = pitch_sp(1:end-nk);
% t = t(1:end-nk);
%t = t(1+nk:end);

%% Identification

%n4sid
nx = 2; %model order
sys = n4sid(data,nx,'DisturbanceModel','none','N4Weight','MOESP');

figure(1)
pos_fig = [0 0 1920 1080];
set(gcf,'Position',pos_fig)
compare(data,sys)
image_path =strcat('Images/',file_name,'-ID.png');
saveas(gcf,image_path)

%% Load some verification data 

file_name = '2021-12-07-16-48-15-3-1-4';
file_path =strcat(file_name,'.mat');
image_path =strcat('Images/',file_name,'.png');

load(file_path)

figure(2)
pos_fig = [0 0 1920 1080];
set(gcf,'Position',pos_fig)
hold on 
plot(t, vel_sp);
plot(t, vel_sys);
grid on
legend("Setpoint","System Response")
title(file_name)
xlabel("time [s]")
ylabel("velocity [m/s]")
saveas(gcf,image_path)

ver_data3 = iddata(vel_sys,vel_sp,Ts);
%% Compare system response to verification data 

figure(2)
pos_fig = [0 0 1920 1080];
set(gcf,'Position',pos_fig)
compare(ver_data3,sys)
image_path =strcat('Images/',file_name,'-ID.png');
saveas(gcf,image_path)