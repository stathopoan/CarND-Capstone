
class LowPassFilter(object):
    def __init__(self, tau, ts):
        self.a = 1. / (tau / ts + 1.)
        self.b = tau / ts / (tau / ts + 1.);

        self.last_val = 0.
        self.ready = False

    def get(self):
        return self.last_val

    def filt(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val


class SimpleLowPassFilter(object):
    def __init__(self, weight):
        assert .0 <= weight <= 1.
        self.weight = weight
        self.last_value = .0
        self.ready = False

    def filt(self, value):
        if self.ready:
            value = self.weight * value + (1.-self.weight)* value
        else:
            self.ready = True

        self.last_value = value
        return value
