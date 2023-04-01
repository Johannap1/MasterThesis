start = - fan_parameters["size"] / 2
end = fan_parameters["size"] / 2
max_strength = fan_parameters["strength"]
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