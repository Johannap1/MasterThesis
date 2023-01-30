clc;
clear;
%% Load the file path and settings
% Select the correct file path
bag = rosbag("forces_simplesim_x_field_2023-01-25-10-09-40.bag")
%% Kino path
bSel = select(bag,'Topic','/resilient_planner_node/kino_path');
msgStructs = readMessages(bSel,'DataFormat','struct');

x_path_mat = [];
y_path_mat = [];

x_0 = zeros(1,250);

for i = 1:118
    x_path = [];
    y_path = [];
    a = size(msgStructs{i,1}.Poses)
    for j = 1:a(2)
        x_path = [x_path,msgStructs{i,1}.Poses(j).Pose.Position.X];
        y_path = [y_path,msgStructs{i,1}.Poses(j).Pose.Position.Y];
    end
    x_path(numel(x_0)) = 0;
    y_path(numel(x_0)) = 0;
    x_path_mat = [x_path_mat;x_path];
    y_path_mat = [y_path_mat;y_path];
end
csvwrite(strcat('forces_data_x/','x_path_mat.csv'),x_path_mat);
csvwrite(strcat('forces_data_x/','y_path_mat.csv'),y_path_mat);

%% Traveled path 
bSel = select(bag,'Topic','/drone_hummingbird/state');
msgStructs = readMessages(bSel,'DataFormat','struct');

x = [];
y = [];

for i = 1:951
    x = [x,msgStructs{i,1}.Pose.Pose.Position.X];
    y = [y,msgStructs{i,1}.Pose.Pose.Position.Y];
end
csvwrite(strcat('forces_data_x/','x.csv'),x');
csvwrite(strcat('forces_data_x/','y.csv'),y');

%% Elliposoids

bSel = select(bag,'Topic','/resilient_planner_node/ellipsoids');
msgStructs = readMessages(bSel,'DataFormat','struct');

x_pos_mat = [];
y_pos_mat = [];
x_scale_mat = [];
y_scale_mat = [];

for i = 1:943
    x_pos = [];
    y_pos = [];
    x_scale = [];
    y_scale = [];
    for j = 1:20
        x_pos = [x_pos ,msgStructs{i,1}.Markers(j).Pose.Position.X];
        y_pos = [y_pos ,msgStructs{i,1}.Markers(j).Pose.Position.Y];
        x_scale = [x_scale,msgStructs{i,1}.Markers(j).Scale.X];
        y_scale = [y_scale,msgStructs{i,1}.Markers(j).Scale.Y];
    end
    x_pos_mat = [x_pos_mat;x_pos];
    y_pos_mat = [y_pos_mat;y_pos];
    x_scale_mat = [x_scale_mat;x_scale];
    y_scale_mat = [y_scale_mat;y_scale];
end
csvwrite(strcat('forces_data_x/','x_pos_ell.csv'),x_pos_mat);
csvwrite(strcat('forces_data_x/','y_pos_ell.csv'),y_pos_mat);
csvwrite(strcat('forces_data_x/','x_scale_ell.csv'),x_scale_mat);
csvwrite(strcat('forces_data_x/','y_scale_ell.csv'),y_scale_mat);