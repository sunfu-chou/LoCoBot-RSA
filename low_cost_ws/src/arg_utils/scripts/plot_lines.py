import numpy as np
import matplotlib.pyplot as plt

import add_path
from arg_utils.plotting import *

# try xy_plot first
vec1 = np.array([[1, 2], [3, 4], [5, 6]])
vec2 = np.array([[1, 3], [2, 4], [3, 5]])
vec3 = np.array([[1, 4], [2, 5], [3, 6]])
vec4 = np.array([[1, 5], [2, 6], [3, 7]])
xy_plot('xy_plot', 'x', 'y', vec1, 'vec1', vec2, 'vec2', vec3, 'vec3', vec4, 'vec4')
plt.show()

# try xyzt_plot next
vec1 = np.array([[1, 2, 3, 4], [2, 3, 4, 5], [3, 4, 5, 6]])
vec2 = np.array([[1, 3, 4, 5], [2, 4, 5, 6], [3, 5, 6, 7]])
vec3 = np.array([[1, 4, 5, 6], [2, 5, 6, 7], [3, 6, 7, 8]])
vec4 = np.array([[1, 5, 6, 7], [2, 6, 7, 8], [3, 7, 8, 9]])
xyzt_plot('xyzt_plot', vec1, 'vec1', vec2, 'vec2', vec3, 'vec3', vec4, 'vec4')
plt.show()

