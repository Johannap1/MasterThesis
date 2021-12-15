clc;
clear all;

%% Load the Data

file_name = '2021-12-15-09-51-03-1-0-3';
file_path =strcat('YawRate_Exp/',file_name,'.mat');
image_path =strcat('Images/','YawRate_Exp/',file_name,'.png');

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
ylabel("amplitude [rad/s]")
saveas(gcf,image_path)
