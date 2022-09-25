import math
import torch
import gpytorch
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
from sklearn import preprocessing
from mpl_toolkits import mplot3d

# import numpy as np
#
# # Define squared exponential kernel
# params = {'l': [1.0], 'sigma_f': 1.0}
#
#
# def squared_exponential_kernel(xa, xb):
#     k = np.square(params['sigma_f']) * np.exp(-np.square(xa - xb) / 2 * np.square(params['l']))
#     return k
#
# #TODO: Look at Torrente code

x = np.linspace(0,10,11)
y = np.linspace(0,15,16)
(X,Y) = np.meshgrid(x,y)
u = 1.4*np.ones(X.shape)
v = np.ones(X.shape)
q = plt.quiver(X,Y,u,v,angles='xy',scale=10,color='r')
p = plt.quiverkey(q,1,16.5,50,"50 m/s",coordinates='data',color='r')
xl = plt.xlabel("x (km)")
yl = plt.ylabel("y (km)")
plt.show()