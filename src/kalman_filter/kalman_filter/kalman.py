import numpy as np

class KalmanFilter:
    def __init__(self, F, H, Q, R, B, u) -> None:
        self.F = F
        self.H = H
        self.Q = Q
        self.R = R
        self.B = B
        self.u = u
        self.x = 0
        self.z = 0

    def predict(self):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, self.u)
        self.P = np.dot(self.F, np.dot(self.P, np.transpose(self.F))) + self.Q

    def update(self):
        y = self.z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, np.transpose(self.H))) + self.R
        K = np.dot(self.P, np.transpose(self.H)) + np.invert(S) 
        self.x = self.x + np.dot(K, y)
        self.P = np.dot((np.identity(1) - np.dot(K,self.H)), self.P)
        y = self.z - np.dot(self.H, self.x)
