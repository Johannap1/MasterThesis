import sys
import numpy as np
from scipy import ndimage
import matplotlib.pyplot as plt
import math

#load file path
file_path = sys.argv[1]

# Define the dimensions of the wind field
# Start point of the wind field in x, y and z (for now assume no terrain profile)
min_x = -10
min_y = -10
min_z = 0

# Dimension of the wind field in x, y and z
dim_x = 20
dim_y = 20
dim_z = 10

# Dimension of the grid in x, y and z
# Has to be uneven for fan option
n_x = 51
n_y = 51
n_z = 2

# Calculate grid resolution
res_x = dim_x/n_x
res_y = dim_y/n_y
res_z = dim_z/n_z

print(res_x,res_y, res_z)

# Vertical spacing
vertial_spacing = "uniform" # TODO: implement other options
vertical_spacing_factors = np.empty([n_z], dtype=float)
if vertial_spacing == "uniform":
    for i in range(n_z):
        vertical_spacing_factors[i] = i/(n_z-1)

# Bottom and top z (for now assume no terrain profile)
bottom_z = np.zeros([n_x*n_y])
top_z = dim_z*np.ones([n_x*n_y])

# Fill in wind field
# wind_field_type = "uniform" # TODO: implement other options
# wind_direction = {"x": 1.0, "y": 0.0, "z": 0.0}

wind_field_type = "fan"
# fans = {
#     'fan1': {
#         "x_pos": -10,
#         "y_pos":  0,
#         "size": 2,
#         "x_dir": 0,
#         "y_dir": 1,
#         "strength": 5,
#         "spread": 0.25,
#         "decrease":0.25,
#     },
#     'fan2': {
#         "x_pos": 0,
#         "y_pos": -10,
#         "size": 2,
#         "x_dir": 1,
#         "y_dir": 0,
#         "strength": 3,
#         "spread": 0.15,
#         "decrease": 0.15,
#     }
# }

fans = {
    'fan1': {
        "x_pos": -10,
        "y_pos":  0,
        "size": 2,
        "x_dir": 0,
        "y_dir": 1,
        "strength": 5,
        "spread": 0.25,
        "decrease":0.25,
    }
}


# fans = {
#     'fan1': {
#         "x_pos": -7,
#         "y_pos": 7,
#         "size": 3,
#         "x_dir": -0.5,
#         "y_dir": 0.8,
#         "strength": 5,
#         "spread": 0.25,
#         "decrease":0.25,
#     },
#     'fan2': {
#         "x_pos": 2,
#         "y_pos": 2,
#         "size": 5,
#         "x_dir": -1,
#         "y_dir": -1,
#         "strength": 3,
#         "spread": 0.15,
#         "decrease": 0.15,
#     },
#     'fan3': {
#         "x_pos": 3,
#         "y_pos": -7,
#         "size": 1,
#         "x_dir": -0.6,
#         "y_dir": -0.9,
#         "strength": 3,
#         "spread": 0.15,
#         "decrease": 0.15,
#     },
#     'fan4': {
#         "x_pos": 8,
#         "y_pos": -9,
#         "size": 7,
#         "x_dir": 0.3,
#         "y_dir": -0.2,
#         "strength": 4,
#         "spread": 0.2,
#         "decrease": 0.1,
#     }
# }

if wind_field_type == "uniform":
    u = wind_direction["x"] * np.ones([n_x*n_y*n_z])
    v = wind_direction["y"] * np.ones([n_x * n_y * n_z])
    w = wind_direction["z"] * np.ones([n_x * n_y * n_z])

if wind_field_type == "fan":
    u = np.zeros([n_x * n_y * n_z])
    v = np.zeros([n_x * n_y * n_z])
    w = np.zeros([n_x * n_y * n_z])
    for fan, fan_parameters in fans.items():
        mat = np.empty([0])
        # v = np.empty([0])
        start = - fan_parameters["size"] / 2
        end = fan_parameters["size"] / 2
        max_strength= fan_parameters["strength"]
        for i in range(n_y):
            x_fit = np.array([start, 0, end])
            y_fit = np.array([0, max_strength, 0])
            z = np.polyfit(x_fit, y_fit, 2)
            p = np.poly1d(z)
            y_in = np.arange(min_y + res_y / 2, min_y + dim_y + res_y / 2, res_y)
            y_out = p(y_in)
            y_out[y_out < 0] = 0
            mat = np.append(mat, y_out)
            start = start - res_x * fan_parameters["spread"]
            end = end + res_x * fan_parameters["spread"]
            max_strength = max_strength - res_x * fan_parameters["decrease"]
        wind_mat = np.reshape(mat, (n_x, n_y))
        rot = math.atan2(fan_parameters["y_dir"],fan_parameters["x_dir"]) * 180 / math.pi
        wind_mat_rot = ndimage.rotate(wind_mat, rot, reshape=False)
        max_value = np.where(wind_mat_rot == np.amax(wind_mat_rot))
        x_shift = math.floor(fan_parameters["y_pos"] - (max_value[0] * res_x + min_x))/res_x
        y_shift = math.floor(fan_parameters["x_pos"] - (max_value[1] * res_y + min_y))/res_y
        print(x_shift, y_shift)
        wind_mat_shift = ndimage.shift(wind_mat_rot, (x_shift, y_shift))

        wind_norm = np.sqrt(fan_parameters["x_dir"]**2 + fan_parameters["y_dir"]**2)
        u_fan = wind_mat_shift.flatten('F')*fan_parameters["x_dir"]/wind_norm
        v_fan = wind_mat_shift.flatten('F') * fan_parameters["y_dir"] / wind_norm
        u_fan = np.tile(u_fan, n_z)
        v_fan = np.tile(v_fan, n_z)
        u = u + u_fan
        v = v + v_fan

# Save the data to file
np.set_printoptions(threshold=sys.maxsize, linewidth=sys.maxsize)
array_name_list = ["min_x", "min_y", "n_x", "n_y", "res_x", "res_y", "vertical_spacing_factors", "bottom_z", "top_z", "u", "v", "w"]
arraylist = [min_x, min_y, n_x, n_y, res_x, res_y, vertical_spacing_factors, bottom_z, top_z, u, v, w]
save_file = open(file_path,"w")

for i in range(12):
    save_file.write(str(array_name_list[i]))
    save_file.write(":\n")
    # if (type(array_name_list[i]).__module__)=='numpy':
    #     save_file.write(np.array2string(arraylist[i]))
    # else:
    save_file.write(str(arraylist[i]))
    save_file.write("\n")
save_file.close()

# Clean up the file from unnecessary brackets
read_file = open(file_path,"r")
data = read_file.read()
data = data.replace("[","")
data = data.replace("]","")
save_file = open(file_path,"w")
save_file.write(data)
save_file.close()

# Visualization
fig = plt.figure(figsize=(10, 3))
ax1, ax2, ax3 = fig.subplots(1, 3)
ax1.imshow(wind_mat, cmap='gray')
ax1.set_axis_off()
ax2.imshow(wind_mat_rot, cmap='gray')
ax2.set_axis_off()
ax3.imshow(wind_mat_shift, cmap='gray')
ax3.set_axis_off()
fig.set_tight_layout(True)
plt.show()