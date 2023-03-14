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

# Vertical spacing
vertial_spacing = "uniform"
vertical_spacing_factors = np.empty([n_z], dtype=float)
if vertial_spacing == "uniform":
    for i in range(n_z):
        vertical_spacing_factors[i] = i/(n_z-1)

# Bottom and top z (for now assume no terrain profile)
bottom_z = np.zeros([n_x*n_y])
top_z = dim_z*np.ones([n_x*n_y])

wind_field_type = "fan"
fans = {
    'fan1': {
        "x_pos": -12,
        "y_pos": 0,
        "size": 2,
        "x_dir": 0,
        "y_dir": 1,
        "strength": 6,
        "spread": 0.25,
        "decrease": 0.25,
    }
    'fan2': {
            "x_pos": -12,
            "y_pos": 0,
            "size": 2,
            "x_dir": 0,
            "y_dir": 1,
            "strength": 4,
            "spread": 0.15,
            "decrease": 0.15,
        }
}

if wind_field_type == "fan":
    u = np.zeros([n_x * n_y * n_z])
    v = np.zeros([n_x * n_y * n_z])
    w = np.zeros([n_x * n_y * n_z])
    for fan, fan_parameters in fans.items():
        mat = np.empty([0])
        # Retrieve fan size and strength from defined parameters
        start = - fan_parameters["size"] / 2
        end = fan_parameters["size"] / 2
        max_strength= fan_parameters["strength"]
        # For one row of the grid
        for i in range(n_y):
            # Fit a quadratic polynomial to the start and end point
            # with the maximum at the maximum strength value
            x_fit = np.array([start, 0, end])
            y_fit = np.array([0, max_strength, 0])
            z = np.polyfit(x_fit, y_fit, 2)
            p = np.poly1d(z)
            # Define grid row and fit the data
            y_in = np.arange(min_y + res_y / 2, min_y + dim_y + res_y / 2, res_y)
            y_out = p(y_in)
            # Correct negative values to zero
            y_out[y_out < 0] = 0
            # Append data row to grid
            mat = np.append(mat, y_out)
            # Spread the fan for next row
            start = start - res_x * fan_parameters["spread"]
            end = end + res_x * fan_parameters["spread"]
            # Decrease the strength for next row
            max_strength = max_strength - res_x * fan_parameters["decrease"]
        # Reshape to marix format
        wind_mat = np.reshape(mat, (n_x, n_y))
        # Rotate matrix to have the correct direction of the fan
        rot = math.atan2(fan_parameters["y_dir"],fan_parameters["x_dir"]) * 180 / math.pi
        wind_mat_rot = ndimage.rotate(wind_mat, rot, reshape=False)
        # Shift to the correct starting position
        max_value = np.where(wind_mat_rot == np.amax(wind_mat_rot))
        x_shift = math.floor(fan_parameters["y_pos"] - (max_value[0] * res_x + min_x))/res_x
        y_shift = math.floor(fan_parameters["x_pos"] - (max_value[1] * res_y + min_y))/res_y
        wind_mat_shift = ndimage.shift(wind_mat_rot, (x_shift, y_shift))
        # Normalize the wind
        wind_norm = np.sqrt(fan_parameters["x_dir"]**2 + fan_parameters["y_dir"]**2)
        u_fan = wind_mat_shift.flatten('F')*fan_parameters["x_dir"]/wind_norm
        v_fan = wind_mat_shift.flatten('F')*fan_parameters["y_dir"]/wind_norm
        u_fan = np.tile(u_fan, n_z)
        v_fan = np.tile(v_fan, n_z)
        u = u + u_fan
        v = v + v_fan