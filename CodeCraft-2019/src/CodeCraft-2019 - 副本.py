import logging
import sys
import numpy as np
import pandas as pd
from Car import Car
from Cross import Cross
from Road import Road
from path import Map


# logging.basicConfig(level=logging.DEBUG,
#                     filename='../logs/CodeCraft-2019.log',
#                     format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
#                     datefmt='%Y-%m-%d %H:%M:%S',
#                     filemode='a')


# 判断是否全部车辆都到达的函数
def isArr(car_list):
    for i in range(len(car_list)):
        if car_list[i].isloc == False:
            return False
    return True

# 初始化地图
def initmap(cross, roads):
    value = np.zeros([cross.__len__(), cross.__len__()])
    for i in range(cross.__len__()):
        for j in range(cross.__len__()):
            value[i][j] = float("inf")
    for i in range(roads.__len__()):
        road = roads[i]
        count_car = 1
        velocity = 1
        minv = 10
        # 获取这条路上的车数量，和他们速度平均值
        for i in range(road.channel.__len__()):
            for j in range(road.channel[0].__len__()):
                if not road.channel[i][j] == 0:
                    velocity = velocity+road.channel[i][j].spd
                    if road.channel[i][j].spd < minv:
                        minv = road.channel[i][j].spd    # 道路中速度最小值
                    count_car = count_car+1
        velocity = velocity//count_car
        v = 100 * road.lth // road.spd//road.chlnum//pow(minv, 2)//pow(velocity,2) + 10*road.lth//road.spd  # 大体上估计的一个公式
        value[road.fr][road.to] = v
    return value

def main():
    # if len(sys.argv) != 5:
    #     logging.info('please input args: car_path, road_path, cross_path, answerPath')
    #     exit(1)
    #
    # car_path = sys.argv[1]
    # road_path = sys.argv[2]
    # cross_path = sys.argv[3]
    # answer_path = sys.argv[4]
    #
    # logging.info("car_path is %s" % (car_path))
    # logging.info("road_path is %s" % (road_path))
    # logging.info("cross_path is %s" % (cross_path))
    # logging.info("answer_path is %s" % (answer_path))

# to read input file
    car_data = pd.read_csv("car.txt")
    cross_data = pd.read_csv("cross.txt")
    road_data = pd.read_csv("road.txt")
    car_list = []
    cross_list = []
    road_list = []
    # 将车都存入一个列表
    for i in range(car_data.__len__()):
        car = car_data.loc[i]
        car_list.append(
            Car(id=int(car[0][1:]), fr=int(car[1]) - 1, to=int(car[2]) - 1, speed=int(car[3]), plt=int(car[4][:-1])))
    # 将路口存入一个列表
    for i in range(cross_data.__len__()):
        cross = cross_data.loc[i]
        cross_list.append(Cross(id=int(car[0][1:]), rid1=int(cross[1]), rid2=int(cross[2]), rid3=int(cross[3]),
                                rid4=int(cross[4][:-1]), mapid=i))
    # 将路存入一个列表
    for i in range(road_data.__len__()):
        road = road_data.loc[i]
        road_list.append(
            Road(id=int(road[0][1:]), lth=int(road[1]), spd=int(road[2]), chlnum=int(road[3]), fr=int(road[4]) - 1,
                 to=int(road[5]) - 1))
        if int(road[6][:-1]) == 1:  # 如果是双向的就再加上
            road_list.append(
                Road(id=int(road[0][1:]), lth=int(road[1]), spd=int(road[2]), chlnum=int(road[3]), fr=int(road[5]) - 1,
                     to=int(road[4]) - 1))
    # 初始化地图权值
    map_value = initmap(cross=cross_list, roads=road_list)
    # 初始化车辆  每辆车用迪杰斯特拉规划一下路径
    for car in car_list:
        car.map = map_value
        car.Dijkstra()
# process
    time = 0  # 计时器
    mmap = Map(crosses=cross_list, cars=car_list, roads=road_list, time=time)  # 创建地图
    # 当所有的车还未到达终点，一直更新地图
    while isArr(car_list) == False:
        mmap.next()
        # 隔一辆更新一辆车的路线
        # new_map = initmap(cross_list, road_list)
        # if time % 3 == 0:
        #     for i in range(car_list.__len__()):
        #         if i % 3 == 0:
        #             car_list[i].map = new_map
        #             car_list[i].Dijkstra(isInit=False)
        time = time + 1
        print(time)

# to write output file
    mmap.cross_road()
    result = ''
    for car in car_list:
        car.realroad.insert(0, car.realplt)
        car.realroad.insert(0, car.id)
        string = "(" + ",".join([str(x) for x in car.realroad]) + ")"
        result = string+"\n"
    with open("answer.txt", 'w') as f:
        f.write(result)



if __name__ == "__main__":
    main()