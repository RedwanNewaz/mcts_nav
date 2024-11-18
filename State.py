import math
import random
import numpy as np

class State:
    def __init__(self, x, y, theta, v, w):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.w = w

    def __str__(self):
        return f"x: {self.x}, y: {self.y}"

    def __repr__(self):
        return f"x: {self.x}, y: {self}"

