import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import decimate

#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')

EARTH_FIELD = 51.7 

def apply_compensation(sensor, bias, sfs, a, b, c):
    return np.array([[sfs[0], a, b],
                     [a, sfs[1], c],
                     [b, c, sfs[2]]]).dot(np.vstack(sensor + bias)).flatten()



def err(X, data, solve_hard, solve_diag, solve_off_diag, hard_iron=None, diag=None, mix=None):
    xctr = 0
    if solve_hard:
        bias = X[xctr:3]
        xctr += 3
    else:
        if hard_iron is not None:
            bias = hard_iron
        else:
            bias = np.array([0, 0, 0])

    if solve_diag:
        sfs = X[xctr:xctr+3] #[1.0, X[3], X[4]]
        xctr += 3
    else:
        if diag is not None:
            sfs = diag
        else:
            sfs = np.ones((3,))
    
    if solve_off_diag: 
        a, b, c = X[xctr:xctr+3]
        xctr += 3
    else:
        if mix is not None:
            a, b, c = mix
        else:
            a, b, c = np.zeros((3, ))
    
      #  X[3]

    compensated = np.apply_along_axis(lambda x: apply_compensation(x, bias, sfs, a, b, c), 1, data)
    
    return np.linalg.norm(compensated, axis=1) - EARTH_FIELD
    

def calibrate(data, solve_hard: bool, solve_diag: bool, solve_off_diag: bool, 
              hard_iron=None, diag=None, mix=None):

    X0 = []
    if solve_hard and (hard_iron is not None):
        X0 += hard_iron
    elif solve_hard:
        X0 += [0, 0, 0]

    if solve_diag and (diag is not None):
        X0 += diag
    elif solve_diag:
        X0 += [1, 1, 1]        
    
    if solve_off_diag and (mix is not None):
        X0 += mix
    elif solve_off_diag:
        X0 += [0, 0, 0]

    #print("X0", X0)
    X = least_squares(err, X0, args=(data, solve_hard, solve_diag, solve_off_diag, hard_iron, diag, mix)) 
    return X


def cal_sequence(data):
    hard_iron = [0, 0, 0]
    diag = [1, 1, 1]
    mix = [0, 0, 0]
    for _ in range(5):
        #print(hard_iron)
        #print(diag, mix)
        X = calibrate(data, True, False, False, hard_iron=hard_iron, diag=diag, mix=mix)
        if X.success:
            hard_iron = X.x.tolist()
        else:
            raise Exception()
        X = calibrate(data, False, True, True, hard_iron=hard_iron, diag=diag, mix=mix)
        if X.success:
            tmp = X.x.tolist()
            diag = tmp[:3]
            mix = tmp[3:]
        else:
            raise Exception()
        print(X.cost)
    return hard_iron, diag, mix
            


def plot(data, hard_iron, soft_iron):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(data[:,0], data[:,1], data[:, 2])

    compensated = np.apply_along_axis(lambda x: apply_compensation(x, hard_iron, soft_iron[:3], *soft_iron[3:]), 1, data)

    ax.scatter(compensated[:,0], compensated[:,1], compensated[:,2])
    u = np.linspace(0, 2 * np.pi, 40)
    v = np.linspace(0, np.pi, 40)

    x = EARTH_FIELD * np.outer(np.cos(u), np.sin(v))
    y = EARTH_FIELD * np.outer(np.sin(u), np.sin(v))
    z = EARTH_FIELD * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_wireframe(x, y, z)

#[ 2.84646440e+01,  1.91644186e+01,  1.91047554e+00, -3.44977241e-02,
#        5.92371583e-03, -1.02868528e-01]

    