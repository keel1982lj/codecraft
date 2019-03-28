class Car(object):
    def __init__(self, id, fr, to, speed,  plt, map=None, isloc=False):
        self.id = id
        self.fr = fr
        self.to = to
        self.speed = speed    # 车辆最大速度
        self.spd = speed    # spd实际速度
        self.plt = plt
        self.isloc = isloc
        self.map = map
        self.planpath = []
        self.realpath = []
        self.time = 0
        self.realroad=[]
        self.realplt=plt

    # dijkstra 算法规划路径
    def Dijkstra(self, isInit=True):
        dis = {}              # 当前节点距离每个节点的距离
        visited = []
        _min_dis = None
        _min_dis_point = None
        fr = self.fr
        if isInit == False:
            fr = self.planpath[1]
        # 先初始化
        for i in range(len(self.map)):     # map指的是各个路口到各个路口的权值矩阵
            dis[i] = [self.map[fr][i], []]
        for i in range(len(dis)):
            sort_dis = sorted(dis.items(), key=lambda item: item[1][0])  # 找到dis中距离起始点距离最小的点
            for p, d in sort_dis:
                if p not in visited:
                    _min_dis_point = p
                    _min_dis = d[0]
                    visited.append(p)
                    break
            for j in range(len(self.map)):
                if self.map[_min_dis_point][j] < float("inf"):
                    update = _min_dis + self.map[_min_dis_point][j]
                    if dis[j][0] > update:
                        dis[j][0] = update
                        dis[j][1] = dis[_min_dis_point][1][:]
                        dis[j][1].append(_min_dis_point)
        value = dis[self.to][1]
        value.extend([self.to])
        value.insert(0, fr)
        if isInit == False:
            value.insert(0, self.planpath[0])
        self.planpath = value  # 返回[mapid, mapid, ....]

