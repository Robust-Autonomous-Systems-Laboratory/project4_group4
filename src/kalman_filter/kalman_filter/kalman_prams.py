import numpy as np
class Kf_prams():
    def __init__(self,dt):
        self.F = np.array([[1,dt,0,0,0,0,0,0,0],
                    [0,1,dt,0,0,0,0,0,0],
                    [0,0,1,0,0,0,0,0,0],
                    [0,0,0,1,dt,0,0,0,0],
                    [0,0,0,0,1,dt,0,0,0],
                    [0,0,0,0,0,1,0,0,0],
                    [0,0,0,0,0,0,1,dt,0],
                    [0,0,0,0,0,0,0,1,dt],
                    [0,0,0,0,0,0,0,0,1]])
            
        self.B = np.identity(9)   
        self.H = np.identity(9)
        self.Q = np.identity(9)
        self.R = np.identity(9)
        self.x0 = np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0]])
        self.P0 = np.identity(9)