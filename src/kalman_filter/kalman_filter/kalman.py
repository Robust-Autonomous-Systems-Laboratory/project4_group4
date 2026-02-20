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
        #print("u = \n",u)
        #print("estimated x = \n", x)
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
        residual = abs(z - np.dot(self.H, self.x))
        #print("z = \n", z)
        #print("updated x = \n", x)
        #print("\n")
        return x, residual, self.P

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
        self.k = 1
        self.beta = 2
        self.n = len(self.x)
        self.n_sigma = (2 * self.n) + 1
        self.lambda_= ((self.alpha**2) * (self.n + self.k)) - self.n
        self.xbar = np.transpose(self.x)
        self.sigma = []
        self.covariance_weights = np.zeros(self.n_sigma)
        self.mean_weights = np.zeros(self.n_sigma)
        self.covariance_weights,self.mean_weights = self.get_weights()

    def get_weights(self):
        print("n = ",self.n)
        print("lambda = ",self.lambda_)
        self.covariance_weights[0] = (self.lambda_ / (self.n + self.lambda_)) + (1 - (self.alpha**2) + self.beta)
        self.mean_weights[0] = (self.lambda_ / (self.n + self.lambda_))
        for i in range(1,self.n_sigma):
            self.covariance_weights[i] = 1/(2*(self.n + self.lambda_))
            self.mean_weights[i] = 1/(2*(self.n + self.lambda_))
        #print("mean weights = ",self.mean_weights)
        #print("c weights = ",self.covariance_weights)
        return self.covariance_weights, self.mean_weights

    def get_sigma_points(self):
        self.L = np.linalg.cholesky(np.dot((self.n + self.lambda_),self.Pxx))
        #print("L = ", self.L)
        sigmaT = np.zeros((self.n_sigma, self.n))
        sigmaT[0] = self.xbar
        for i in range(1,self.n):
            #print("i = ",i)
            sigmaT[i] = self.xbar + self.L[i]
            sigmaT[i+self.n] = self.xbar - self.L[i]
        self.sigma = np.transpose(sigmaT)
        return self.sigma

    def predict(self):
        self.sigma = self.get_sigma_points()
        self.y = np.zeros(np.shape(self.sigma))
        temp = np.zeros(np.shape(np.transpose(self.sigma)))
        for i in range(self.n_sigma):
            temp[i] = (np.dot(self.F,np.transpose(self.sigma)[i]))
        self.y = np.transpose(temp)
        self.ybar = np.zeros(self.n)
        for i in range(self.n):
            for j in range(self.n_sigma):
                self.ybar[i] = self.ybar[i] + np.dot(self.mean_weights[j], self.y[i,j])
        self.Pyy = np.zeros((self.n,self.n))
        for i in range(self.n_sigma):#this loop taken from listed github, thanks to user balghane
            delta = self.y.T[i] - self.ybar
            delta = np.atleast_2d(delta)
            self.Pyy += self.covariance_weights[i] * np.dot(delta.T, delta)
        self.Pyy = self.Pyy + self.Q
        #print("y = ",self.y)
        #print("ybar = ",self.ybar)
        return self.ybar
        
    def update(self,z,H):
        ztrue = z
        self.H = H
        temp = np.zeros(np.shape(np.transpose(self.y)))
        for i in range(self.n_sigma):
            temp[i] = (np.dot(self.H,np.transpose(self.y)[i]))
        self.z = np.transpose(temp)
        self.zbar = np.zeros(self.n)
        for i in range(self.n):
            for j in range(self.n_sigma):
                self.zbar[i] = self.zbar[i] + np.dot(self.mean_weights[j], self.z[i,j])
        self.Pzz = np.zeros((self.n,self.n))
        for i in range(self.n_sigma):#this loop taken from listed github, thanks to user balghane
            delta = self.z.T[i] - self.zbar
            delta = np.atleast_2d(delta)
            self.Pzz += self.covariance_weights[i] * np.dot(delta.T, delta)
        self.Pzz = self.Pzz + self.R
        self.Pyz = np.zeros((self.n,self.n))
        for i in range(self.n_sigma):
            delta1 = self.y.T[i] - self.ybar
            delta1 = np.atleast_2d(delta1)
            delta2 = self.z.T[i] - self.zbar
            delta2 = np.atleast_2d(delta1)
            self.Pyz += self.covariance_weights[i] * np.dot(delta1.T, delta2)

        self.K = np.dot(self.Pyz,np.linalg.inv(self.Pzz))
        self.residual = abs(ztrue - np.transpose(self.zbar))
        self.x = np.transpose([self.ybar]) - np.dot(self.K, np.transpose(self.residual))
        self.P = self.Pyy - np.dot(self.K,np.dot(self.Pzz,np.transpose(self.K)))
        #print("x = ",self.x)
        return self.x, self.residual, self.P
    
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
        #print("u = \n",u)
        #print("estimated x = \n", x)
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
        residual = abs(z - np.dot(self.H, self.x))
        #print("z = \n", z)
        #print("updated x = \n", x)
        #print("\n")
        return x, residual, self.P

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


