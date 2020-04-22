import numpy as np 


class BaseController(object):
    def __init__(self, rocket):
        self.r = rocket

    def getAction(self):
        self.r.states
        action = None
        return action

    def train(self):
        raise NotImplementedError

    def test(self):
        while self.r.states[0] > 0:
            action = self.getAction()

            self.r._update(0.3)

    def cost(self):
        raise NotImplementedError

