import numpy as np

class PID(object):
    """A simple PID controller."""

    def __init__(
        self,
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        setpoint=0.0,
        proportional_on_measurement=False,
        differential_on_measurement=False,
        sample_freq = 1000.0,
    ):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint

        self.proportional_on_measurement = proportional_on_measurement
        self.differential_on_measurement = differential_on_measurement

        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_output = None
        self._last_error = None
        self._last_input = None

        self.sample_freq = sample_freq
        self.reset()

    def __call__(self, input_, dt=None):
        """
        Update the PID controller.
        
        """
        dt = 1/self.sample_freq
        
        # Compute error terms
        error = self.setpoint - input_        
        d_input = input_ - (self._last_input if (self._last_input is not None) else input_)
        
        d_error = error - (self._last_error if (self._last_error is not None) else error)

        # Compute the proportional term
        if not self.proportional_on_measurement:
            # Regular proportional-on-error, simply set the proportional term
            self._proportional = self.Kp * error
        else:
            self._proportional -= self.Kp * d_input

        # Compute integral and derivative terms
        self._integral += self.Ki * error * dt
            
        # self._integral = _clamp(self._integral, self.integral_limits)  # Avoid integral windup

        if not self.differential_on_measurement:
            # self._derivative = self.Kd * d_error_smooth
            self._derivative = self.Kd * d_error / dt
        else:
            self._derivative = -self.Kd * d_input / dt
            # self._derivative = -self.Kd * d_input / dt

        # Compute final output
        output = self._proportional + self._integral + self._derivative

        # Keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_error = error

        return output


    @property
    def components(self):
        """
        The P-, I- and D-terms from the last computation as separate components as a tuple. Useful
        for visualizing what the controller is doing or when tuning hard-to-tune systems.
        """
        return self._proportional, self._integral, self._derivative

    def reset(self):
        """
        Reset the PID controller internals.

        This sets each term to 0 as well as clearing the integral, the last output and the last
        input (derivative calculation).
        """
        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_output = None
        self._last_input = None
