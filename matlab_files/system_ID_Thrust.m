clc;
clear all;

%% Specify File Paths

file_name = '2021-12-13-16-55-50-2-1-4';
file_path =strcat(file_name,'.mat');
image_path =strcat('Images/',file_name,'.png');

%% Load and visualize the data

load(file_path)

figure(1)
pos_fig = [0 0 1920 1080];
plot(t,acc)
hold on 
plot(t,thrust)
saveas(gcf,image_path)

%% Determine hovering thrust

acc_hover = acc(((-0.01 < acc) & (acc < 0.01)))
thrust_hover = thrust(((-0.01 < acc) & (acc < 0.01)))
hover_thrust = mean(thrust_hover)
thrust_norm = thrust-hover_thrust;


figure(3)
plot(t, thrust_norm)
hold on 
plot(t, acc)

%% Compute Acceleration Thrust Ratio

acc_thrust_ratio = acc./thrust_norm;
acc_thrust_ratio = median(acc_thrust_ratio(acc_thrust_ratio > 0.0))

figure(4)
pos_fig = [0 0 1920 1080];
plot(t, acc)
hold on 
plot(t, thrust_norm*acc_thrust_ratio)
image_path =strcat('Images/',file_name,'-ID.png');
saveas(gcf,image_path)

y = acc;
y_hat = thrust_norm*acc_thrust_ratio;
RMSE = sqrt(1/length(y)*sum((y-y_hat).^2));
goodnessOfFit(y,y_hat,'NRMSE')
%% Same assuming a time delay

Ts = 0.02;
data = iddata(acc, thrust_norm,Ts);
k = delayest(data);
figure(5)
plot(t(1:end-k), thrust_norm(1:end-k))
hold on 
plot(t(1:end-k), acc(k+1:end))

acc_thrust_ratio = acc(k+1:end)./thrust_norm(1:end-k);
acc_thrust_ratio_ = median(acc_thrust_ratio(acc_thrust_ratio > 0.0))

figure(4)
plot(t(1:end-k), thrust_norm(1:end-k)*acc_thrust_ratio_)
hold on 
plot(t(1:end-k), acc(k+1:end))

y = acc;
y_hat = thrust_norm*acc_thrust_ratio_;
RMSE = sqrt(1/length(y)*sum((y-y_hat).^2));
goodnessOfFit(y,y_hat,'NRMSE')

%% Validation

file_name = '2021-12-14-15-02-09-1-0-5';
file_path =strcat(file_name,'.mat');
image_path =strcat('Images/',file_name,'.png');
load(file_path)

figure(1)
plot(t,acc)
hold on 
plot(t,thrust)
saveas(gcf,image_path)

cmd_acc = acc;
cmd_thrust = (thrust-hover_thrust)*acc_thrust_ratio_;
figure(4)
plot(t, cmd_acc)
hold on 
plot(t, cmd_thrust)
image_path =strcat('Images/',file_name,'-ID.png');
saveas(gcf,image_path)

NRMSE = goodnessOfFit(cmd_acc,cmd_thrust,'NRMSE')