import os
import sys
sys.path.append(os.getcwd())
from src.estimator import Estimator
import numpy as np

thetas = np.array([0, .8, 1.2])
r_h = np.array([[1,1], [.5,.7], [.3,.5]])
# r_hs = np.array([[0, 0], []])
print(Estimator.regress(thetas, r_h))