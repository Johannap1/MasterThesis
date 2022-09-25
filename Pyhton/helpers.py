import numpy as np
import math

class WindCalculator:
    def __init__(self,windfield_params):
        self.windfield_params = windfield_params
        self.windfield_params = windfield_params
        self.use_custom_windfield = self.windfield_params['use_custom_windfield']
        self.wind_direction = self.windfield_params['wind_direction']
        self.wind_velocity = self.windfield_params['wind_velocity']
        wind_norm = np.sqrt(self.wind_direction[0] ** 2 + self.wind_direction[1] ** 2 + self.wind_direction[2] ** 2)
        self.wind_direction = self.wind_direction / wind_norm
        if self.use_custom_windfield:
            windfield_path= self.windfield_params['windfield_path']
            try:
                f = open(windfield_path)
            except:
                print("File '" + windfield_path + "' does not exist!")
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

            self.min_x_ = data["min_x"]
            self.min_y_ = data["min_y"]
            self.n_x_ = int(data["n_x"])
            self.n_y_ = int(data["n_y"])
            self.res_x_ = data["res_x"]
            self.res_y_ = data["res_y"]
            self.vertical_spacing_factors_ = data["vertical_spacing_factors"]
            self.bottom_z_ = data["bottom_z"]
            self.top_z_ = data["top_z"]
            self.u_vec_ = data["u"]
            self.v_vec_ = data["v"]
            self.w_vec_ = data["w"]

            self.n_z = len(self.vertical_spacing_factors_)

    def linear_interpolation(self, position, values, points):
        value = values[0] + (values[1] - values[0]) / (points[1] - points[0]) * (position - points[0])
        return value

    def bilinear_interpolation(self, position, values, points):
        intermediate_values = np.array([self.linear_interpolation(position[0], values[0:], points[0:]),
                                        self.linear_interpolation(position[0], values[2:], points[2:])])
        value = self.linear_interpolation(position[1], intermediate_values, points[4:])
        return value

    def trilinear_interpolation(self, position, values, points):
        intermediate_values = np.array([self.linear_interpolation(position[2], values[0:], points[0:]),
                                        self.linear_interpolation(position[2], values[2:], points[2:]),
                                        self.linear_interpolation(position[2], values[4:], points[4:]),
                                        self.linear_interpolation(position[2], values[6:], points[6:])])
        value = self.bilinear_interpolation(position[0:],intermediate_values, points[8:])
        return value

    def calculate_windfield(self, link_position):

        if self.use_custom_windfield:
            # Calculate x,y index just smaller than aircraft position or equal to
            x_inf = math.floor((link_position[0] - self.min_x_) / self.res_x_)
            y_inf = math.floor((link_position[1] - self.min_y_) / self.res_y_)

            # In case aircraft is on one of the boundary surfaces at max_x or max_y,
            # decrease x_inf, y_inf by one to have x_sup, y_sup on max_x, max_y.
            if x_inf == self.n_x_ - 1:
                x_inf = self.n_x_ - 2
            if y_inf == self.n_y_ - 1:
                y_inf = self.n_y_ - 2

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
                vertical_factors_columns[i] = (link_position[2] - self.bottom_z_[
                    idx_x[2 * i] + idx_y[2 * i] * self.n_x_]) \
                                              / (self.top_z_[idx_x[2 * i] + idx_y[2 * i] * self.n_x_] -
                                                 self.bottom_z_[idx_x[2 * i] + idx_y[2 * i] * self.n_x_])

            # Find minimal and maximal vertical factor
            vertical_factors_min = min(min(min(
                vertical_factors_columns[0], vertical_factors_columns[1]),
                vertical_factors_columns[2]), vertical_factors_columns[3])
            vertical_factors_max = max(max(max(
                vertical_factors_columns[0], vertical_factors_columns[1]),
                vertical_factors_columns[2]), vertical_factors_columns[3])

            # Check if aircraft is out of wind field or not, and act accordingly.
            if (x_inf >= 0 and y_inf >= 0 and vertical_factors_max >= 0 and
                    x_sup <= (self.n_x_ - 1) and y_sup <= (self.n_y_ - 1) and vertical_factors_min <= 1):
                # Find indices in z-direction for each of the vertices.If link is not
                # within the range of one of the columns, set to lowest or highest two.
                idx_z = [0, int(len(self.vertical_spacing_factors_)) - 1,
                         0, int(len(self.vertical_spacing_factors_)) - 1,
                         0, int(len(self.vertical_spacing_factors_)) - 1,
                         0, int(len(self.vertical_spacing_factors_)) - 1]
                for i in range(n_columns):
                    if vertical_factors_columns[i] < 0:
                        # Link z - position below lowest grid point of that column
                        idx_z[2 * i + 1] = 1
                    elif vertical_factors_columns[i] >= 1:
                        # Link z-position above highest grid point of that column
                        idx_z[2 * i] = len(self.vertical_spacing_factors_) - 2
                    else:
                        # Link z-position between two grid points in that column.
                        for j in range(len(vertical_factors_columns) - 1):
                            if self.vertical_spacing_factors_[j] <= vertical_factors_columns[i] < \
                                    self.vertical_spacing_factors_[j + 1]:
                                idx_z[2 * i] = j
                                idx_z[2 * i + 1] = j + 1
                                break

                # Extract the wind velocities corresponding to each vertex
                wind_at_vertices = np.empty([8, 3])
                for i in range(n_vertices):
                    wind_at_vertices[i, 0] = self.u_vec_[
                        idx_x[i] + idx_y[i] * self.n_x_ + idx_z[i] * self.n_x_ * self.n_y_]
                    wind_at_vertices[i, 1] = self.v_vec_[
                        idx_x[i] + idx_y[i] * self.n_x_ + idx_z[i] * self.n_x_ * self.n_y_]
                    wind_at_vertices[i, 2] = self.w_vec_[
                        idx_x[i] + idx_y[i] * self.n_x_ + idx_z[i] * self.n_x_ * self.n_y_]

                # Extract the relevant coordinate of every point needed for trilinear interpolation
                n_points_interp_z = 8
                n_points_interp_x = 4
                n_points_interp_y = 2
                interpolation_points = np.empty(n_points_interp_x + n_points_interp_y + n_points_interp_z)
                for i in range(n_points_interp_x + n_points_interp_y + n_points_interp_z):
                    if i < n_points_interp_z:
                        interpolation_points[i] = (self.top_z_[idx_x[i] + idx_y[i] * self.n_x_] - self.bottom_z_[
                            idx_x[i] + idx_y[i] * self.n_x_]) \
                                                  * self.vertical_spacing_factors_[idx_z[i]] + self.bottom_z_[
                                                      idx_x[i] + idx_y[i] * self.n_x_]
                    elif n_points_interp_z <= i < n_points_interp_x + n_points_interp_z:
                        interpolation_points[i] = self.min_x_ + self.res_x_ * idx_x[2 * (i - n_points_interp_z)]
                    else:
                        interpolation_points[i] = self.min_y_ + self.res_y_ * idx_y[
                            4 * (i - n_points_interp_z - n_points_interp_x)]

                wind_v = self.trilinear_interpolation(link_position, wind_at_vertices, interpolation_points)
            else:
                print("Drone is outside of specified custom windfield, using default one")
                wind_v = self.wind_direction * self.wind_velocity
        else:
            wind_v = self.wind_direction * self.wind_velocity
        return wind_v

    def return_ground_truth(self):
        mean = []
        x = self.windfield_params['x'].flatten()
        y = self.windfield_params['y'].flatten()
        for i in range(x.size):
            link_position = [x[i], y[i], 2] #Random z parameter for now
            wind_v = self.calculate_windfield(link_position)
            mean.append(wind_v)
        mean = np.array(mean)
        return mean










