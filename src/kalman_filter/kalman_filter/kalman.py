import numpy as np

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

    updated_state = kf.update(z,H)
    print("Updated state:\n", updated_state)
#test()


