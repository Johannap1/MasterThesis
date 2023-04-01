clc;
clear;
%% Load the file path and settings
% Select the correct file path
bag = rosbag("lmpcc_simplesim_cc_xy_field_2023-01-22-11-37-53.bag")
%%
bSel = select(bag,'Topic','lmpcc/planned_trajectory');
msgStructs = readMessages(bSel,'DataFormat','struct');
%% Read data differently 

x_mat = [];
y_mat = [];
sigma_x_mat = [];
sigma_y_mat = [];
sigma_xy_mat = [];
slack_mat = [];

for i = 1:63
    message_x = [];
    message_y = [];
    sigma_x = [];
    sigma_y = [];
    sigma_xy = [];
    slack = [];
    for j = 1:20
        message_x = [message_x,msgStructs{i,1}.Poses(j).Pose.Position.X];
        message_y = [message_y,msgStructs{i,1}.Poses(j).Pose.Position.Y];
        sigma_x = [sigma_x,msgStructs{i,1}.Poses(j).Pose.Orientation.X];
        sigma_y = [sigma_y,msgStructs{i,1}.Poses(j).Pose.Orientation.Y];
        sigma_xy = [sigma_xy,msgStructs{i,1}.Poses(j).Pose.Orientation.W];
        slack = [slack,msgStructs{i,1}.Poses(j).Pose.Position.Z];
    end
    x_mat = [x_mat; message_x];
    y_mat = [y_mat; message_y];
    sigma_x_mat = [sigma_x_mat;sigma_x];
    sigma_y_mat = [sigma_y_mat;sigma_y];
    sigma_xy_mat = [sigma_xy_mat;sigma_xy];
    slack_mat = [slack_mat; slack];
end
csvwrite(strcat('cc_xy_field/','x_mat.csv'),x_mat);
csvwrite(strcat('cc_xy_field/','y_mat.csv'),y_mat);
csvwrite(strcat('cc_xy_field/','sigma_x_mat.csv'),sigma_x_mat);
csvwrite(strcat('cc_xy_field/','sigma_y_mat.csv'),sigma_y_mat);
csvwrite(strcat('cc_xy_field/','sigma_xy_mat.csv'),sigma_xy_mat);
csvwrite(strcat('cc_xy_field/','slack_mat.csv'),slack_mat);
%%
bSel = select(bag,'Topic','drone_hovergames/wind');
ts_state= timeseries(bSel,...
           'Header.Stamp.Sec',...
           'Header.Stamp.Nsec',...
           'Vector.X',...
           'Vector.Y');

wind_x = ts_state.Data(:,3);
wind_y = ts_state.Data(:,4);