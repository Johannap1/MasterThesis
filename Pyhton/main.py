# TODO Test for other inputs as well
#
import math
import sys
import torch
import gpytorch
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
from sklearn import preprocessing
from sklearn.model_selection import train_test_split
from mpl_toolkits import mplot3d
from helpers import WindCalculator
from matplotlib.widgets import Cursor
np.set_printoptions(threshold=sys.maxsize)


class ExactGPModel(gpytorch.models.ExactGP):
    def __init__(self, train_x, train_y, likelihood):
        super(ExactGPModel, self).__init__(train_x, train_y, likelihood)
        self.mean_module = gpytorch.means.ConstantMean()
        self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel())

    def forward(self, x):
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)

def read_data(filepath):
    training_data = pd.read_csv(filepath)
    training_data_array = training_data.to_numpy()
    X = torch.FloatTensor(np.stack((training_data_array[:, 0], training_data_array[:, 1]), axis=-1))
    y1 = torch.FloatTensor(training_data_array[:, 2])
    y2 = torch.FloatTensor(training_data_array[:, 3])
    return X, y1, y2

def plot_input_data(X, y1, y2, title):
    fig, axs = plt.subplots(2, 3)
    axs[0, 0].remove()
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.scatter(X.numpy()[:, 0], X.numpy()[:, 1], y1.numpy())
    axs[0, 1].plot(X.numpy()[:,0], y1.numpy(), 'r*')
    axs[0, 1].set_xlabel("x-direction[m]")
    axs[0, 1].set_ylabel("force in x-direction [m/s^2]")
    axs[0, 2].plot(X.numpy()[:,1], y1.numpy(), 'g*')
    axs[0, 2].set_xlabel("y-direction[m]")
    axs[0, 2].set_ylabel("force in x-direction [m/s^2]")
    axs[1, 0].remove()
    ax4 = fig.add_subplot(2, 3, 4, projection='3d')
    ax4.scatter(X.numpy()[:, 0], X.numpy()[:, 1], y2.numpy())
    axs[1, 1].plot(X.numpy()[:, 0], y2.numpy(), 'r*')
    axs[1, 1].set_xlabel("x-direction[m]")
    axs[1, 1].set_ylabel("force in y-direction [m/s^2]")
    axs[1, 2].plot(X.numpy()[:, 1], y2.numpy(), 'g*')
    axs[1, 2].set_xlabel("y-direction[m]")
    axs[1, 2].set_ylabel("force in y-direction [m/s^2]")
    fig.suptitle(title)
    plt.ioff()
    plt.show()
def plot_wind_field(X, y1, y2, title, res):
    fig, ax = plt.subplots(1, 1)
    plt.axis('equal')
    plt.grid(linestyle=':')
    try:
        q = plt.quiver(X.numpy()[::res, 0], X.numpy()[::res, 1], y1.numpy()[::res], y2.numpy()[::res], scale=2.54, color='r',
                       units='width', scale_units="inches", width=0.003)
    except:
        q = plt.quiver(X[::res, 0], X[::res, 1], y1[::res], y2[::res], scale=2.54, color='r',
                       units='width', scale_units="inches", width=0.003)
    plt.title(title)
    plt.ioff()
    plt.show()

def train_GP(X_train, y_train, module):
    if module == "GPytorch":
        # initialize likelihood and model
        likelihood = gpytorch.likelihoods.GaussianLikelihood()
        model = ExactGPModel(X_train, y_train, likelihood)
        # Use the adam optimizer
        optimizer = torch.optim.Adam(model.parameters(), lr=0.1)  # Includes GaussianLikelihood parameters
        # "Loss" for GPs - the marginal log likelihood
        mll = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood, model)
        training_iter = 50
        for i in range(training_iter):
            # Zero gradients from previous iteration
            optimizer.zero_grad()
            # Output from model
            output = model(X_train)
            # Calc loss and backprop gradients
            loss = -mll(output, y_train)
            loss.backward()
            print('Iter %d/%d - Loss: %.3f   lengthscale: %.3f   noise: %.3f' % (
                i + 1, training_iter, loss.item(),
                model.covar_module.base_kernel.lengthscale.item(),
                model.likelihood.noise.item()
            ))
            optimizer.step()
        return model, likelihood

    if module == "sklearn":
        #Scale the input data
        scaler = preprocessing.StandardScaler().fit(X_train)
        X_scaled = scaler.transform(X_train)
        kernel = 1 * RBF(length_scale=10.0, length_scale_bounds=(1e-2, 1e2))
        model = GaussianProcessRegressor(kernel=kernel, alpha=0.01 ** 2, n_restarts_optimizer=9)
        model.fit(X_scaled, y_train)
        return model, scaler

def test_GP(X_test, module, model, likelihood=None, scaler=None):
    if module == "GPytorch":
        model.eval()
        likelihood.eval()
        observed_pred = likelihood(model(X_test))
        mean_pred = observed_pred.mean.detach().numpy()
        lower, upper = observed_pred.confidence_region()
        lower_pred = lower.detach().numpy()
        upper_pred = upper.detach().numpy()
    if module == "sklearn":
        X_test_scaled = scaler.transform(X_test)
        mean_pred, std_pred = model.predict(X_test_scaled, return_std=True)
        lower_pred = mean_pred - 1.96 * std_pred
        upper_pred = mean_pred + 1.96 * std_pred
    return mean_pred, upper_pred, lower_pred

def create_grid(start, stop, res):
    x = np.linspace(start, stop, res)
    y = np.linspace(start, stop, res)
    x, y = np.meshgrid(x, y)
    X = torch.FloatTensor(np.stack((x.flatten(), y.flatten()), axis=-1))
    return X, x, y
def plot_surface(x, y, mean, upper, lower, bounds):
    mean_grid = mean.reshape(x.shape)
    fig = plt.figure()
    if bounds:
        upper_grid = upper.reshape(x.shape)
        lower_grid = lower.reshape(x.shape)
        ax = fig.gca(projection='3d')
        ax.plot_surface(x, y, mean_grid)
        ax.plot_surface(x, y, upper_grid, color='C0', alpha=0.2)
        ax.plot_surface(x, y, lower_grid, color='C0', alpha=0.2)
    else:
        ax1 = fig.subplots(1, 1)
        ax1 = plt.contourf(x, y, mean_grid)
        plt.colorbar(ax1)
    plt.ioff()
    plt.show()



#Read and plot the csv data
X, y1, y2 = read_data('/home/johanna/MasterThesis/Wind_data/sim_data/lmpcc_windsimplesim_fan_x_dir_2022-09-07-10-45-44.csv')
# plot_input_data(X, y1, y2, "Input data for Lemniscate trajectory with wind in x and y direction")
plot_wind_field(X, y1, y2, "Measured Disturbance",8)
# Test train split
X_train, X_test, y1_train, y1_test = train_test_split(X, y1, test_size=0.2, random_state=1)
X_train, X_test, y2_train, y2_test = train_test_split(X, y2, test_size=0.2, random_state=1)
# Train gaussian process
# GP_model_y1, scaler_y1 = train_GP(X_train, y1_train, "sklearn")
# GP_model_y2, scaler_y2 = train_GP(X_train, y2_train, "sklearn")
GP_model_y1, likelihood_y1 = train_GP(X_train, y1_train, "GPytorch")
GP_model_y2, likelihood_y2 = train_GP(X_train, y2_train, "GPytorch")

# Plot wind field at the test points
# mean_y1, upper_y1, lower_y1 = test_GP(X_test, "sklearn", GP_model_y1, None, scaler_y1)
# mean_y2, upper_y2, lower_y2 = test_GP(X_test, "sklearn", GP_model_y2,  None, scaler_y2)
mean_y1, upper_y1, lower_y1 = test_GP(X_test, "GPytorch", GP_model_y1, likelihood_y1)
mean_y2, upper_y2, lower_y2 = test_GP(X_test, "GPytorch", GP_model_y2, likelihood_y2)
plot_wind_field(X_test, mean_y1, mean_y2, "Predicted Disturbance",1)

# Plot wind field in grid area
X_grid, x_grid, y_grid = create_grid(-10,9.9,51)
mean_y1, upper_y1, lower_y1 = test_GP(X_grid, "GPytorch", GP_model_y1, likelihood_y1)
mean_y2, upper_y2, lower_y2 = test_GP(X_grid, "GPytorch", GP_model_y2, likelihood_y2)
# mean_y1, upper_y1, lower_y1 = test_GP(X_grid, "sklearn", GP_model_y1, None, scaler_y1)
# mean_y2, upper_y2, lower_y2 = test_GP(X_grid, "sklearn", GP_model_y2, None, scaler_y2)
plot_wind_field(X_grid, mean_y1, mean_y2, "Predicted Disturbance",8)

# Plot 2D surface mean only
plot_surface(x_grid, y_grid, mean_y1, upper_y1, lower_y1, False)
plot_surface(x_grid, y_grid, mean_y2, upper_y2, lower_y2, False)

# Plot 2D surfaces with uncertainty
plot_surface(x_grid, y_grid, mean_y1, upper_y1, lower_y1, True)
plot_surface(x_grid, y_grid, mean_y2, upper_y2, lower_y2, True)

# Ground thruth
wind_field_parameters = {'use_custom_windfield': True, 'wind_direction': np.array([1.0, 1.0, 0.0]), 'wind_velocity': 1.0, 'x':x_grid, 'y':y_grid, 'windfield_path':'/home/johanna/uav_mpcc/src/model_sim/include/custom_wind_fields/fan_2_fans_random.txt'}
wind_calculator = WindCalculator(wind_field_parameters)
mean_gt = wind_calculator.return_ground_truth()
plot_surface(x_grid, y_grid, mean_gt[:,0], 0, 0, False)
plot_surface(x_grid, y_grid, mean_gt[:,1], 0, 0, False)
plot_wind_field(X_grid, mean_gt[:,0], mean_gt[:,1], "Predicted Disturbance",8)
print(mean_gt)