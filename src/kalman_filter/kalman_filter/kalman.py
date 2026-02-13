import numpy as np

class KalmanFilter:
    def __init__(self, F, H, Q, R, B, x_0, P_0) -> None:
        """
        Docstring for __init__
        
        :param self: Description
        :param F: 3x3 matrix
        :param H: Description
        :param Q: Description
        :param R: Description
        :param B: 3x3 matrix
        :param x_0: 3x1 vector
        :param P_0: Description
        """
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
        return self.x

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
        residual = z - np.dot(self.H, self.x)
        return self.x, residual

def test(): 
    F = np.array([[1,1,0,0,0,0,0,0],
                  [0,1,1,0,0,0,0,0],
                  [0,0,1,0,0,0,0,0],
                  [0,0,0,1,1,0,0,0],
                  [0,0,0,0,1,1,0,0],
                  [0,0,0,0,0,1,0,0],
                  [0,0,0,0,0,0,1,1],
                  [0,0,0,0,0,0,0,1]])
    
    B = np.array([[0,0,0,0,0,0,0,0],
                  [0,1,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0],
                  [0,0,0,1,0,0,0,0],
                  [0,0,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,1]])
    
    H = np.array([[1,2,3]])
    Q = np.array([[1,2,3],[0,1,2],[1,2,3]])
    R = np.array([[1,2,3],[0,1,2],[1,2,3]])
    x0 = np.array([[1],[1],[1]])
    P0 = np.array([[1],[1],[1]])
    kf = KalmanFilter(F, H, Q, R, B, x0, P0)
    u = np.array([[1],[1],[1]])
    z = np.array([[1],[1],[1]])

    predicted_state = kf.predict(u)
    print("Predicted state:\n", predicted_state)

    updated_state = kf.update(z)
    print("Updated state:\n", updated_state)
test()

