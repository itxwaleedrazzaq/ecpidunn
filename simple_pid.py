# PID Controller Class
class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, setpoint, measurement):
        error = setpoint - measurement
        P = self.Kp * error
        self.integral += error * self.dt
        I = self.Ki * self.integral
        derivative = (error - self.prev_error) / self.dt
        D = self.Kd * derivative
        self.prev_error = error
        control_signal = P + I + D
        return control_signal
