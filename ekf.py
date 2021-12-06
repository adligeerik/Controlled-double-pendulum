import numpy as np
import sympy as sp

class ExtendedKalmanFilter:
    def __init__(self, x0: np.ndarray, P: np.ndarray, dt: float, Q: np.ndarray, R: np.ndarray, A_non_lin: sp.Matrix, dx: sp.Matrix, C: np.ndarray, input_symbol: sp.Symbol):
        self.x_hat = x0.reshape(len(x0), 1)
        self.P = P
        self.dt = dt
        self.Q = Q
        self.R = R
        self.I = np.eye(len(x0))
        self.C = C

        self.u = input_symbol

        self.dx = dx
        self.A_non_lin = A_non_lin
        self.fj = A_non_lin.jacobian(dx)

        self.hj = (C @ dx).jacobian(dx)

    def predict(self, u: float):
        self.x_hat = self.x_hat + self.dt * self.system(x = self.x_hat, u = u)
        F = self.Fjacobian(x = self.x_hat, dt = self.dt)
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z: np.ndarray):
        y_err = z - self.h(self.x_hat)
        H = self.Hjacobian(self.x_hat)
        S = H @ self.P @ H.T + self.R

        K = (self.P @ H.T) * np.linalg.inv(S)
        self.x_hat = self.x_hat + K.ravel() * y_err
        self.P = (self.I - K @ H) @ self.P

    def Fjacobian(self, x: np.ndarray, dt: float) -> np.ndarray:
        val = {self.dx[0]: x[0], self.dx[1]: x[1], self.dx[2]: x[2], self.dx[3]: x[3]}
        df = np.eye(3) + dt * self.fj.xreplace(val)
        return df

    def system(self, x: np.ndarray, u: float) -> np.ndarray:
        val = {self.dx[0]: x[0], self.dx[1]: x[1], self.dx[2]: x[2], self.dx[3]: x[3], self.u: u}
        x_dot = self.A_non_lin.xreplace(val)
        print( np.array(x_dot).astype(np.float64))
        return np.array(x_dot).astype(np.float64)

    def h(self, x: np.ndarray) -> float:
        return self.C @ x

    def Hjacobian(self) -> float:
        return self.hj
