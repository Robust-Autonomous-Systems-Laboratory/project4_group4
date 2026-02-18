import numpy as np
from numpy import float64

class KalmanFilter:
    def __init__(self, F, H, Q, R, B, x_0, P_0) -> None:
        self.F = F
        self.H = H
        self.Q = Q
        self.R = R
        self.B = B
        self.x = x_0
        self.P = P_0

    def predict(self,u):
        """
        Part 1 of a Kalman filter. Predict the state at time k using data from k-1.
        
        :param self: Self
        :param u: Control Vector
        :return: Predicted next state
        """
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(self.F, np.dot(self.P, np.transpose(self.F))) + self.Q
        x = self.x
        print("u = \n",u)
        print("estimated x = \n", x)
        return x

    def update(self,z):
        """
        Docstring for update
        
        :param self: Self
        :param z: Observation Vector
        :return: Estimation of the current state
        """
        
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, np.transpose(self.H))) + self.R
        K = np.dot(np.dot(self.P, np.transpose(self.H)), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot((np.identity(self.P.shape[0]) - np.dot(K,self.H)), self.P)
        x = self.x
        residual = z - np.dot(self.H, self.x)
        print("z = \n", z)
        print("updated x = \n", x)
        print("\n")
        return x, residual

class UnscentedKalmanFilter:
    """
    Docstring for UnscentedKalmanFilter
    Loop structure based on https://github.com/balghane/pyUKF/blob/master/ukf.py
    """
    def __init__(self, F, H, Q, R, x_0, P_0, deltaT) -> None:
        self.F = F
        self.H = H
        self.Q = Q
        self.R = R
        self.x = x_0
        self.Pxx = P_0
        self.deltaT = deltaT
        self.alpha = 0.001
        self.k = 0 
        self.beta = 2
        self.n = len(self.x[0])
        self.n_sigma = (2 * self.n) + 1
        self.lambda_= + pow(self.alpha, 2) * (self.n + self.k) - self.n
        self.covariance_weights,self.mean_weights = self.get_weights()

    def get_weights(self):
        self.covariance_weights[0] = (self.lambda_ / (self.n + self.lambda_)) + (1 - pow(self.alpha, 2) + self.beta)
        self.mean_weights[0] = (self.lambda_ / (self.n + self.lambda_))
        for i in range(1,(2*self.n)):
            self.covariance_weights[i] = 1/(2*(self.n - self.lambda_))
            self.mean_weights[i] = 1/(2*(self.n - self.lambda_))
        return self.covariance_weights, self.mean_weights

    def get_sigma_points(self):
        self.L = np.linalg.cholesky(np.dot((self.n + self.lambda_),self.Pxx))
        self.xbar = np.average(self.x)
        sigmaT = []
        sigmaT[0] = self.xbar
        for i in range(1,self.n):
            sigmaT[i] = self.xbar + self.L
            sigmaT[i+self.n] = self.xbar - self.L
        self.sigma = np.transpose(sigmaT)
        return self.sigma

    def predict(self):
        self.sigma = self.get_sigma_points()
        self.y = np.dot(self.F,self.sigma)
        for i in range(2*self.n):
            self.ybar = self.ybar + np.dot(self.mean_weights[i], self.y[i])
        for i in range(2*self.n):
            self.Pyy = self.Pyy + np.dot(self.covariance_weights[i],np.dot((self.y[i] - self.ybar),np.transpose((self.y[i] - self.ybar)))) + self.Q
        return self.y

    def update(self,z,H):
        self.z = z
        self.H = H
        self.z = np.dot(self.H,self.y)
        for i in range(2*self.n):
            self.zbar = self.zbar + np.dot(self.mean_weights[i], self.z[i])
        for i in range(2*self.n):
            self.Pzz = self.Pzz + np.dot(self.covariance_weights[i],np.dot((self.z[i] - self.zbar),np.transpose((self.z[i] - self.zbar)))) + self.R
        for i in range(2*self.n):
            self.Pyz = self.Pyz + np.dot(self.covariance_weights[i],np.dot((self.y[i] - self.ybar),np.transpose((self.z[i] - self.zbar))))
        self.K = np.dot(self.Pyz,np.linalg.inv(self.Pzz))
        self.residual = self.z - self.zbar
        self.x = self.ybar - np.dot(self.K, self.residual)
        self.P = self.Pyy - np.dot(self.K,np.dot(self.Pzz,np.transpose(self.K)))
        return self.x, self.P
    
class ExtendedKalmanFilter:
    def __init__(self, F, H, Q, R, B, x_0, P_0) -> None:
        self.F = F
        self.H = H
        self.Q = Q
        self.R = R
        self.B = B
        self.x = x_0
        self.P = P_0

    def predict(self,u):
        """
        Part 1 of a Kalman filter. Predict the state at time k using data from k-1.
        
        :param self: Self
        :param u: Control Vector
        :return: Predicted next state
        """
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(self.F, np.dot(self.P, np.transpose(self.F))) + self.Q
        x = self.x
        print("u = \n",u)
        print("estimated x = \n", x)
        return x

    def update(self,z,H):
        """
        Docstring for update
        
        :param self: Self
        :param z: Observation Vector
        :return: Estimation of the current state
        """
        self.H = H
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, np.transpose(self.H))) + self.R
        K = np.dot(np.dot(self.P, np.transpose(self.H)), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot((np.identity(self.P.shape[0]) - np.dot(K,self.H)), self.P)
        x = self.x
        residual = z - np.dot(self.H, self.x)
        print("z = \n", z)
        print("updated x = \n", x)
        print("\n")
        return x, residual

def test():
    # matrices taken from
    # https://www.geeksforgeeks.org/python/kalman-filter-in-python/
    F = np.array([[1, 1], [0, 1]])
    B = np.array([[0.5], [1]])
    H = np.array([[1, 0]])
    Q = np.array([[1, 0], [0, 1]])
    R = np.array([[1]])
    x0 = np.array([[0], [1]])
    P0 = np.array([[1, 0], [0, 1]])
    u = np.array([[1]])
    z = np.array([[1]])
    kf = KalmanFilter(F, H, Q, R, B, x0, P0)

    predicted_state = kf.predict(u)
    print("Predicted state:\n", predicted_state)

    updated_state = kf.update(z)
    print("Updated state:\n", updated_state)
#test()


