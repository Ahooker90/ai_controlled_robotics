#!/usr/bin/env python

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def update(self, error, current_time):
        delta_time = current_time - self.last_time if self.last_time else 0.0
        if delta_time > 0.0:
            self.integral += error * delta_time
            derivative = (error - self.previous_error) / delta_time
        else:
            derivative = 0.0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        self.last_time = current_time

        return output

