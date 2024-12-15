class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        """
        Initializes the PID controller.

        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        :param setpoint: Desired target value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.omin = -30
        self.omax = 30

        # Internal state
        self.previous_error = 0
        self.integral = 0

    def compute(self, measured_value, dt):
        """
        Compute the output of the PID controller.

        :param measured_value: Current measured value
        :param dt: Time elapsed since the last update (in seconds)
        :return: Control output
        """
        # Calculate error
        error = self.setpoint - measured_value

        # Proportional term
        proportional = self.kp * error

        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral

        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt

        # Update state
        self.previous_error = error

        # Calculate output
        output = proportional + integral + derivative
        return max(self.omin, min(self.omax, output))
