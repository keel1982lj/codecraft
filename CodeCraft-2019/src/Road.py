import numpy as np

class Road(object):
    def __init__(self, id, lth, spd, chlnum, fr, to):
        self.id = id
        self.lth = lth
        self.spd = spd
        self.chlnum = chlnum
        self.fr = fr
        self.to = to
        self.channel = [[0 for i in range(lth)] for j in range(chlnum)]    # 车道
        self.time = 0