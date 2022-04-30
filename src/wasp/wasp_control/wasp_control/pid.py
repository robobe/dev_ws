import time

class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):

        self.__kp = P
        self.__ki = I
        self.__kd = D