class KalmanFilter:
    def __init__(self, R=1, Q=10, Xt=0, Xt_prev=0, Pt_prev=1):
        self.R = R
        self.Q = Q
        self.Xt = Xt
        self.Xt_prev = Xt_prev
        self.Pt_prev = Pt_prev

    def update(self, sensor_data):
        self.Xt_update = self.Xt_prev
        self.Pt_update = self.Pt_prev + self.Q

        self.Kt = self.Pt_update / (self.Pt_update + self.R)
        self.Xt = self.Xt_update + (self.Kt * (sensor_data - self.Xt_update))
        self.Pt = (1 - self.Kt) * self.Pt_update

        self.Xt_prev = self.Xt
        self.Pt_prev = self.Pt

        return self.Xt