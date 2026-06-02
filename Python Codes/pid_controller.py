class PIDController:
    def __init__(self, Kp, Ki, Kd, dt, output_min, output_max):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, set_point, measurement):
        error = set_point - measurement
        derivative = (error - self.prev_error) / self.dt

        tentative = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        if self.output_min < tentative < self.output_max:
            self.integral += error * self.dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        output = max(self.output_min, min(self.output_max, output))

        self.prev_error = error
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
