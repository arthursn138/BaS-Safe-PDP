"""
BaS Augmentation
Based on Hassan's work, to be used with CASADI
Still modular, everything is done thinking about the cartpole example with only one constraint
Arthur Scaquetti do Nascimento
arthursn.138@gmail.com
"""

import numpy as np
from casadi import *

class BaS:
    def __init__(self, env, limit=1, n_bas=1):
        self.dynamics = env.f
        self.states = env.X
        self.limit = limit
        self.n_bas = n_bas

    def safety_function(self):
        h = self.limit ** 2 - self.dynamics[0] ** 2
        h_fn = casadi.Function('h_fn', [self.states], [h])
        # hx = jacobian(self.h_fn, [self.limit, self.dynamics], [h])
        return h, h_fn

    def inverse_barrier(self, h_fn):
        B = 1/h_fn
        B_fn = casadi.Function('B_fn', [self.states], [self.B])
        return B, B_fn

    # def augmented_dyn(self):
    #


    # dyn = env.X + dt * env.f