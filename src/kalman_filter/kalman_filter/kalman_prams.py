import numpy as np
def kf_prams(dt):
    F = np.array([[1,dt,0,0,0,0,0,0,0],
                  [0,1,dt,0,0,0,0,0,0],
                  [0,0,1,0,0,0,0,0,0],
                  [0,0,0,1,dt,0,0,0,0],
                  [0,0,0,0,1,dt,0,0,0],
                  [0,0,0,0,0,1,0,0,0],
                  [0,0,0,0,0,0,1,dt,0],
                  [0,0,0,0,0,0,0,1,dt],
                  [0,0,0,0,0,0,0,0,1]])
        
    B = np.identity(9)   
    H = np.identity(9)
    Q = np.identity(9)
    R = np.identity(9)
    x0 = np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0]])
    P0 = np.identity(9)
    return F,H,Q,R,B,x0,P0