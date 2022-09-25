import sys
import numpy as np
import math

if len(sys.argv) < 2:
    print("usage: python visualize_custom_wind_field.py {input_file}.txt")
    exit(1)

try:
    f = open(sys.argv[1])
except:
    print("File '" + sys.argv[1] + "' does not exist!")
    exit(1)

name = ""
data = dict()
for i in range(26):
    val = f.readline()
    if i % 2 == 0:
        val = str(val)
        val = val.replace(" ", "")
        val = val.replace("\n", "")
        val = val.replace(":", "")
        name = val
    else:
        line = np.fromstring(str(val), sep=" ")
        if (len(line) == 1):
            line = line[0]
        data[name] = line

min_x_ = data["min_x"]
min_y_ = data["min_y"]
n_x_ = int(data["n_x"])
n_y_ = int(data["n_y"])
res_x_ = data["res_x"]
res_y_ = data["res_y"]
vertical_spacing_factors_ = data["vertical_spacing_factors"]
bottom_z_ = data["bottom_z"]
top_z_ = data["top_z"]
u_vec_ = data["u"]
v_vec_ = data["v"]
w_vec_ = data["w"]

n_z = len(vertical_spacing_factors_)

# Get current position of the aircraft
# TODO: replace with actual position
x = np.array([0, -9.58, 0, 0.0, 0.0, 0.0, 0.0, 0.0])
link_position = [x[0],x[1],x[2]]

# Calculate x,y index just smaller than aircraft position or equal to
x_inf = math.floor((x[0] - min_x_) / res_x_)
y_inf = math.floor((x[1] - min_y_) / res_y_)

# In case aircraft is on one of the boundary surfaces at max_x or max_y,
# decrease x_inf, y_inf by one to have x_sup, y_sup on max_x, max_y.
if x_inf == n_x_ - 1:
    x_inf = n_x_ - 2
if y_inf == n_y_ - 1:
    y_inf = n_y_ - 2

# Calculate x,y index just larger than aircraft position
x_sup = x_inf + 1
y_sup = y_inf + 1

# Save grid points enclosing the aircraft in an array
n_vertices = 8
idx_x = np.array([x_inf, x_inf, x_sup, x_sup, x_inf, x_inf, x_sup, x_sup])
idx_y = np.array([y_inf, y_inf, y_inf, y_inf, y_sup, y_sup, y_sup, y_sup])



# Find vertical factors in each of the four surrounding
# grid columns, and their minimal/maximal value.
n_columns = 4
vertical_factors_columns = np.empty(n_columns)
for i in range(n_columns):
    vertical_factors_columns[i] = (x[2] - bottom_z_[idx_x[2 * i] + idx_y[2 * i] * n_x_]) \
                                  /(top_z_[idx_x[2 * i] + idx_y[2 * i] * n_x_] -
                                    bottom_z_[idx_x[2 * i] + idx_y[2 * i] * n_x_])
print(vertical_factors_columns)
# Find minimal and maximal vertical factor
vertical_factors_min = min(min(min(
    vertical_factors_columns[0], vertical_factors_columns[1]),
    vertical_factors_columns[2]), vertical_factors_columns[3])
vertical_factors_max = max(max(max(
      vertical_factors_columns[0], vertical_factors_columns[1]),
      vertical_factors_columns[2]), vertical_factors_columns[3])

# Check if aircraft is out of wind field or not, and act accordingly.
if (x_inf >= 0 and y_inf >= 0 and vertical_factors_max >= 0 and
        x_sup <= (n_x_ - 1) and y_sup <= (n_y_ - 1) and vertical_factors_min <= 1):
    # Find indices in z-direction for each of the vertices.If link is not
    # within the range of one of the columns, set to lowest or highest two.
    idx_z = [0, int(len(vertical_spacing_factors_)) - 1,
             0, int(len(vertical_spacing_factors_)) - 1,
             0, int(len(vertical_spacing_factors_)) - 1,
             0, int(len(vertical_spacing_factors_)) - 1]
    for i in range(n_columns):
        if vertical_factors_columns[i] < 0:
            # Link z - position below lowest grid point of that column
            idx_z[2 * i + 1] = 1
        elif vertical_factors_columns[i] >= 1:
            # Link z-position above highest grid point of that column
            idx_z[2 * i] = len(vertical_spacing_factors_) - 2
        else:
            # Link z-position between two grid points in that column.
            for j in range(len(vertical_factors_columns) -1):
                if vertical_spacing_factors_[j] <= vertical_factors_columns[i] < vertical_spacing_factors_[j + 1]:
                    idx_z[2 * i] = j
                    idx_z[2 * i + 1] = j + 1
                    break

print(idx_x,idx_y, idx_z)

# Extract the wind velocities corresponding to each vertex
wind_at_vertices = np.empty([8,3])
for i in range(n_vertices):
    wind_at_vertices[i, 0] = u_vec_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_]
    wind_at_vertices[i, 1] = v_vec_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_]
    wind_at_vertices[i, 2] = w_vec_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_]


# Extract the relevant coordinate of every point needed for trilinear interpolation
n_points_interp_z = 8
n_points_interp_x = 4
n_points_interp_y = 2
interpolation_points = np.empty(n_points_interp_x + n_points_interp_y + n_points_interp_z)
for i in range(n_points_interp_x + n_points_interp_y + n_points_interp_z):
    if i < n_points_interp_z:
        interpolation_points[i] = (top_z_[idx_x[i] + idx_y[i] * n_x_] - bottom_z_[idx_x[i] + idx_y[i] * n_x_])\
                                  * vertical_spacing_factors_[idx_z[i]] + bottom_z_[idx_x[i] + idx_y[i] * n_x_]
    elif n_points_interp_z <= i < n_points_interp_x + n_points_interp_z:
        interpolation_points[i] = min_x_ + res_x_ * idx_x[2 * (i - n_points_interp_z)]
    else:
        interpolation_points[i] = min_y_ + res_y_ * idx_y[4 * (i - n_points_interp_z - n_points_interp_x)]

print(interpolation_points)
print(wind_at_vertices)
print(link_position)



def linear_interpolation(position, values, points):
    print("position:", position)
    print("values:", values)
    print("points:", points)
    value = values[0] + (values[1] - values[0]) / (points[1] - points[0]) * (position - points[0])
    print("linear value:", value)
    return value

def bilinear_interpolation(position, values, points):
    intermediate_values = np.array([linear_interpolation(position[0], values[0:], points[0:]),
                                    linear_interpolation(position[0], values[2:], points[2:])])
    value = linear_interpolation(position[1], intermediate_values, points[4:])
    return value

def trilinear_interpolation(position, values, points):
    intermediate_values = np.array([linear_interpolation(position[2], values[0:], points[0:]),
                                    linear_interpolation(position[2], values[2:], points[2:]),
                                    linear_interpolation(position[2], values[4:], points[4:]),
                                    linear_interpolation(position[2], values[6:], points[6:])])
    value = bilinear_interpolation(position[0:],intermediate_values, points[8:])
    return value

test_value = trilinear_interpolation(link_position, wind_at_vertices, interpolation_points)
print(test_value)
#
#
# def bilinear_interpolation(position, values, points):
#     intermediate_values = [linear_interpolation(position[0], values[0], points[0]),
#                            linear_interpolation(position[0], values[2], points[2])]
#     value = linear_interpolation(position[1], intermediate_values, points[4])
#     return value
#
#
# def trilinear_interpolation(link_position, values, points):
#     position = [link_position[0], link_position[1], link_position[2]]
#     intermediate_values = [linear_interpolation(position[2], values[0], points[0]),
#                           linear_interpolation(position[2], values[2], points[2]),
#                           linear_interpolation(position[2], values[4], points[4]),
#                           linear_interpolation(position[2], values[6], points[6])]
#     value = bilinear_interpolation(position[0], intermediate_values, points[8])
#     return value
#
# wind_velocity = trilinear_interpolation(link_position, wind_at_vertices, interpolation_points)