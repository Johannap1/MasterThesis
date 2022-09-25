c1 = csvread("sim_data/simplecontrol_windsimplesim_2_fans_random_tsp_path_2022-09-25-13-14-59.csv")
c2 = csvread("sim_data/simplecontrol_windsimplesim_2_fans_random_tsp_path_2022-09-25-14-55-09.csv")
%c3 = csvread("sim_data/lmpcc_windsimplesim_fan_xy_random_line_path_-5_2022-09-14-14-47-46.csv")

c_combined = [c1; c2]; % c3]
csvwrite("sim_data/simplecontrol_windsimplesim_2_fans_random_tsp_path_combined.csv",c_combined)
