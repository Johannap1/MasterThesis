# General 
import sys
import math
import time
import numpy as np
# import pandas as pd
#Plotting
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib import cm
#GPyTorch
import torch
import gpytorch
from torch.utils.data import TensorDataset, DataLoader
from gpytorch.models import ApproximateGP
from gpytorch.variational import CholeskyVariationalDistribution
from gpytorch.variational import VariationalStrategy

class GPModel(ApproximateGP):
    def __init__(self, inducing_points):
        variational_distribution = CholeskyVariationalDistribution(inducing_points.size(0))
        variational_strategy = VariationalStrategy(self, inducing_points, variational_distribution, learn_inducing_locations=True)
        super(GPModel, self).__init__(variational_strategy)
        self.mean_module = gpytorch.means.ZeroMean()
        self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel(ard_num_dims=2))
        #self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.MaternKernel(nu=2.5, ard_num_dims=2))

    def forward(self, x):
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)

class Data:
    def __init__(self, X=None, y=None):
        self.X = X
        self.y = y
    def set_data(self, X, y):
        self.X = X
        self.y = y
        self.read_data_torch()
    def read_data_torch(self):
        self.X_train =  torch.FloatTensor(self.X)
        self.y_train =  torch.FloatTensor(self.y)
        self.X_test =  torch.FloatTensor(self.X)
        self.y_test =  torch.FloatTensor(self.y)
    def read_data(self, inp):
        return

data = np.load('/home/johanna/MasterThesis/data/backup_structured_x_wind/wind_data_4.npz')
data_X = data['X']
data_y = data['y']
inducing_points = torch.FloatTensor(data_X[::int(data_y.shape[0]/30), :])
model = GPModel(inducing_points=inducing_points)
state_dict = torch.load('gpytorch_y1_model.pth')
model.load_state_dict(state_dict)

x1 = torch.Tensor([[0, 0]])
observed_pred = (model(x1))
mean = observed_pred.mean.detach().numpy()
covar = observed_pred.covariance_matrix.detach().numpy()
lower, upper = observed_pred.confidence_region()
lower = lower.detach().numpy()
upper = upper.detach().numpy()
print(mean)
print(covar)

test = np.array([[ 4.0962e-01],
        [ 2.5430e-01],
        [-2.8399e-02],
        [-9.7292e-03],
        [ 2.4176e-03],
        [-8.9946e-04],
        [ 5.3986e-02],
        [ 1.0019e-01],
        [-1.0241e-01],
        [ 5.2700e-03],
        [-3.0922e-03],
        [ 1.8301e-03],
        [-1.1872e-03],
        [ 6.2426e-04],
        [-2.7796e-02],
        [ 2.0165e-01],
        [ 3.7954e-02],
        [ 2.8986e-03],
        [ 6.4584e-03],
        [-2.3925e-03],
        [-1.3657e-02],
        [-3.2625e-04],
        [-1.2722e-02],
        [-7.2355e-03],
        [ 6.5545e-03],
        [-1.6951e-02],
        [-1.4068e-02],
        [ 3.6115e-03],
        [ 3.8613e-04],
        [ 1.0153e-05],
        [-3.6785e-03]])
inducing_values = np.array([[ 2.7275,  1.9786, -0.9070, -0.1217,  0.0270, -0.0542,  0.7861,  2.1987,
        -4.1377,  0.0878,  0.1605, -0.0729,  0.1065, -0.1783,  0.5970,  1.2822,
         0.9710,  1.2329, -0.4235,  0.4875,  0.0908,  0.0194,  0.0187, -0.0866,
        -0.3574,  0.8374, -0.5725, -0.2794,  3.9452, -1.4991,  1.3597]])
sol = np.matmul(inducing_values,test)
print(sol)