# Author: LiShanglin
# Date: 2021/06/07
# Assignment: Maze puzzle
from math import sqrt
import random as rd
from matplotlib import pyplot as plt

MAP = []
mapSize = 10
Step = 0
PATH = []
# 定义以下数值的目的为方便绘图
pass_stat = 0  # 如果是通路，状态为0
srt_stat = 2  # 如果是起点，状态为 2
obs_stat = 1.3  # 如果是障碍物，状态为1.3
end_stat = 3  # 如果是终点，状态为 3


# 定义节点node类，包含了迷宫格中所有必须的信息
class node:
    def __init__(self, x, y, stat):
        self.x = x
        self.y = y
        self.stat = stat
        self.parent = None
        self.g = 0
        self.h = 0
        self.prior = 0
        self.IsPath = 0
        self.searched = 0


def obstacleMap(boolean=False):
    """
    生成障碍物，输入为False为随机生成，输入True为使用固定参数
    默认为False
    :return:(list) obsMap:障碍物地图
    """
    if not boolean:
        obsMap = [[] for _ in range(mapSize+3)]
        for i in range(mapSize+3):
            x = rd.randint(1, mapSize)
            y = rd.randint(1, mapSize)
            if [x, y] in obsMap:
                i = i - 1
                continue
            else:
                obsMap[i] = [x, y]
    else:
        obsMap = [[1, 2], [1, 10], [3, 4], [3, 8], [3, 10],
                  [4, 2], [5, 6], [6, 3], [7, 4], [7, 8], [9, 2], [9, 4], [9, 10]]
        for each in obsMap:
            each[0] -= 1
            each[1] -= 1
    return obsMap


def mapInit(size, startPoint=[2, 2], endPoint=[8, 8], UseDefalutMap=True):
    """
    生成size*size大小的方形地图，并给出规则
    :param (int) size: 地图大小
    :param (list) startPoint:给出起始点坐标[xb,yb]
    :param (list) endPoint:给出终止点坐标[xe,ye]
    :param (bool) UseDefalutMap:是否使用默认地图
    :return:(list) MAP带有标记的地图
    """
    MAP = [[node(0, 0, 0) for _ in range(size)] for i in range(size)]
    obsMap = obstacleMap(UseDefalutMap)
    for x in range(size):
        for y in range(size):
            if [x, y] in obsMap:
                MAP[x][y] = node(x, y, obs_stat)
            elif [x, y] == startPoint:
                MAP[x][y] = node(x, y, srt_stat)
            elif [x, y] == endPoint:
                MAP[x][y] = node(x, y, end_stat)
            else:
                MAP[x][y] = node(x, y, pass_stat)
    print('-' * 30)
    return MAP


def h(thisNode, endNode, method='eu'):
    """
    启发式函数，计算此点到终点的预计损失
    :param (node) thisNode: 当前点[xn,yn]
    :param (node) endNode: 终点[xe,ye]
    :param method: 使用什么样的启发函数，dig：对角距离, mah: 曼哈顿距离, eu:欧几里得距离
    :return:(float) loss:距离代价
    """
    [xn, yn] = [thisNode.x, thisNode.y]
    [xe, ye] = [endNode.x, endNode.y]
    dx = abs(xe - xn)
    dy = abs(ye - yn)
    if method == 'dig':  # 对角距离
        loss = dx + dy + (sqrt(2) - 2) * min(dx, dy)
    elif method == 'mah':  # 曼哈顿距离
        loss = dx + dy
    else:  # 欧几里得距离
        loss = sqrt(dx ** 2 + dy ** 2)
    return loss


def g(dx, dy):
    """
    计算距离上一个节点的移动损失
    :param (int) dx: 相较上一个节点的偏移量x
    :param (int) dy: 相较上一个节点的偏移量y
    :return:(float) res: 距离上一个节点的移动损失
    """
    if dx != 0 and dy != 0:
        res = sqrt(2)
    else:
        res = 1.0
    return res


def observeAround(Node):
    """
    观察周围节点状态,返回可通行节点
    :param (node) Node: 当前节点
    :return: (list) passlist: [node1, node2,...] 当前节点周围的可通行节点
    """
    global MAP
    [xn, yn] = [Node.x, Node.y]
    passlist = []
    for x in range(-1, 2):
        for y in range(-1, 2):
            # 将搜索范围限制在地图范围内
            if y + yn < 0 or y + yn >= mapSize or x + xn < 0 or x + xn >= mapSize:
                continue
            # 当周围节点不是障碍点 且 没有被搜索过 且 不是中心节点的时候，认为是可通行节点
            elif MAP[x + xn][y + yn].stat != obs_stat and MAP[x + xn][y + yn].searched == 0 \
                    and not (xn == 0 and yn == 0):
                MAP[x + xn][y + yn].searched = 1   # 将节点属性设置为“已搜索过”
                MAP[x + xn][y + yn].g = g(x, y) + Node.g  # 计算此节点的g函数值
                passlist.append(MAP[x + xn][y + yn])  # 存入passlist
    return passlist


def UsePrior(Node1):
    """
    排序辅助函数，使用Prior参量
    :param:(node) Node1:操作节点
    :return:
    """
    return Node1.prior


def UseIsPath(Node1):
    """
    排序辅助函数，使用IsPath参量
    :param (node) Node1:操作节点
    :return:
    """
    return Node1.IsPath


def trackPath(Node):
    """
    路径回溯，从终止节点回溯到起始节点，构成路径
    :param (node) Node:当前节点
    :return:
    """
    global Step
    global PATH
    Step += 1
    if Node.parent is not None:
        Node.IsPath = Step
        PATH.append(Node)
        trackPath(Node.parent)
    else:
        PATH.append(Node)
        Node.IsPath = Step


def aStar(size, startPoint=[2, 2], endPoint=[8, 8], UseDefalutMap=True, method='mah'):
    """
    A*算法程序，使用对角距离
    :param (int) size: 地图大小
    :param (list) startPoint:给出起始点坐标[xb,yb]
    :param (list) endPoint:给出终止点坐标[xe,ye]
    :param (bool) UseDefalutMap:是否使用默认地图
    :param (str) method: 使用何种启发式函数，dig,mah,eu
    :return:
    """
    global MAP
    opn = []  # open set
    cls = []  # close set
    flg = 0  # 监测是否搜索到终点
    [xb, yb] = startPoint
    [xe, ye] = endPoint
    MAP = mapInit(size, startPoint=startPoint, endPoint=endPoint,
                  UseDefalutMap=UseDefalutMap)
    srtNode = MAP[xb][yb]
    endNode = MAP[xe][ye]
    opn.append(srtNode)
    while 1:
        if len(opn) > 0:
            opn.sort(key=UsePrior)
            curNode = opn[0]
            if curNode != endNode:
                cls.append(curNode)
                del opn[0]
                passlist = observeAround(curNode)  # 返回可以进行拓展的节点列表
                for eachNode in passlist:
                    if eachNode in cls:
                        continue
                    elif eachNode not in opn:
                        eachNode.parent = curNode
                        eachNode.h = h(eachNode, endNode, method)
                        eachNode.prior = eachNode.g + eachNode.h
                        opn.append(eachNode)
            else:
                trackPath(endNode)
                flg = 1
                break
        else:
            print('open set is empty')
            break
    if flg == 1:
        for each in PATH:
            print('(', each.x, ',', each.y, ')')
        print("Total steps is: ", len(PATH))
        plotTrace(MAP)  # 绘制搜索图
    else:
        print('------Error occurred------')


def transpose(matrix):
    """
    矩阵转置
    :param matrix:输入需要转置的n*n矩阵
    :return:
    """
    return [[matrix[j][i] for j in range(len(matrix))] for i in range(len(matrix[0]))]


def plotTrace(maps):
    """
    画出轨迹
    :param (list) maps:输入地图
    :return:
    """
    MP = [[0 for i in range(mapSize)] for _ in range(mapSize)]
    for x in range(mapSize):
        for y in range(mapSize):
            MP[x][y] = maps[x][y].stat
    MP = transpose(MP)  # 矩阵转置
    plt.subplot(1, 2, 1)
    plt.imshow(MP, cmap=plt.cm.Blues)
    plt.title('Original Map')
    for each in PATH:  # 为路径上深色
        MP[each.y][each.x] = 2
    for x in range(mapSize):  # 为搜索过的路径涂浅色
        for y in range(mapSize):
            if maps[x][y].searched == 1 and maps[x][y] not in PATH:
                MP[y][x] = 0.5
    plt.subplot(1, 2, 2)
    plt.imshow(MP, cmap=plt.cm.Blues)
    plt.title('Searched Path')
    plt.show()


if __name__ == '__main__':
    # startPoint为起始点，endPoint为终点，method为使用的启发函数缩写
    aStar(mapSize, startPoint=[2, 2], endPoint=[8, 8], method='mah', UseDefalutMap=True)
