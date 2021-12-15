clc;
clear all;

%%TODO: Pitch and roll were the other way around!
%% Load the data 

file_name = '2021-12-06-16-09-25-1-0-0';
file_path =strcat(file_name,'.mat');
image_path =strcat('Images/',file_name,'.png');

%%
load(file_path)

figure(1)
pos_fig = [0 0 1920 1080];
set(gcf,'Position',pos_fig)
hold on 
plot(t, pitch_sp);
plot(t, pitch_sys);
grid on
legend("Setpoint","System Response")
title(file_name)
xlabel("time [s]")
ylabel("amplitude [deg]")
%saveas(gcf,image_path)
%% Cut the data sequence if needed

% t_start = 304;
% t_end = 744;
% Ts = 0.1;
% t = t_att_sp(t_start:t_end);
% u = pitch_sp(t_start:t_end);
% y = pitch_sys_resample(t_start:t_end);

%% Store the data in right format

Ts = 0.02;
data = iddata(pitch_sys,pitch_sp,Ts);

%% Estimate the time delay

nk = delayest(data);
%Shift the data accordingly
y = pitch_sys(1+nk:end);
u = pitch_sp(1:end-nk);
t = t(1:end-nk);
%t = t(1+nk:end);
data = iddata(y,u,Ts);

%% Identification

%n4sid
nx = 1; %model order
sys = n4sid(data,nx,'DisturbanceModel','none','N4Weight','MOESP');

figure(1)
pos_fig = [0 0 1920 1080];
set(gcf,'Position',pos_fig)
compare(data,sys)
image_path =strcat('Images/',file_name,'-ID.png');
%saveas(gcf,image_path)
%% Load some verification data 

%file_name = '2021-12-06-17-36-28-2-0-0';
%file_name = '2021-12-06-17-46-44-3-0-0';
%file_name = '2021-12-06-17-56-47-3-0-0';
%file_name = '2021-12-06-18-04-13-2-0-0';
%file_name = '2021-12-07-09-31-18-1-0-0';
%file_name = '2021-12-07-12-23-29-1-1-0';
%file_name = '2021-12-07-12-30-56-2-1-0';
file_name = '2021-12-07-12-33-04-3-1-0';
file_path =strcat(file_name,'.mat');
image_path =strcat('Images/',file_name,'.png');

load(file_path)

figure(2)
pos_fig = [0 0 1920 1080];
set(gcf,'Position',pos_fig)
hold on 
plot(t, pitch_sp);
plot(t, pitch_sys);
grid on
legend("Setpoint","System Response")
title(file_name)
xlabel("time [s]")
ylabel("amplitude [deg]")
%saveas(gcf,image_path)

ver_data8 = iddata(pitch_sys,pitch_sp,Ts);
%% Compare system response to verification data 

figure(2)
pos_fig = [0 0 1920 1080];
set(gcf,'Position',pos_fig)
compare(ver_data8,sys)
image_path =strcat('Images/',file_name,'-ID.png');
%saveas(gcf,image_path)
%% Look at time constants

step(sys) % TOD0: It seems that sampling time is not high enough but we could take the data from the 20Hz one - test that next time