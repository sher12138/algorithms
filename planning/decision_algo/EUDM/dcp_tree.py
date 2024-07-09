"""
DCP-Tree 生成算法
"""
from enum import Enum, unique
import copy
import pprint

class IndexedEnum(Enum):
    @classmethod
    def by_index(cls, index):
        memebers = list(cls)
        if index <0 or index >= len(memebers):
            raise IndexError("index out of range")
        return memebers[index]

class DcpLonAction(IndexedEnum):
    kMaintan = 0
    kAcc = 1
    kDec = 2


class DcpLatAction(IndexedEnum):
    kLaneKeeping = 0
    kLaneChangeLeft = 1
    kLaneChangeRight = 2



class DcpAction:
    def __init__(self, lon, lat, t):
        self.lon = lon
        self.lat = lat
        self.t = t

    def __repr__(self) -> str:
        return "lon: %s, lat: %s, t: %s" % (self.lon, self.lat, self.t)

def AppendActionSequence(seq_in, action, remaining_level):
    seq = copy.deepcopy(seq_in)
    for i in range(remaining_level):
        seq.append(action)
    return seq

def build_tree():
    ongoing_action_ = DcpAction(DcpLonAction.kMaintan, DcpLatAction.kLaneKeeping, 0.0)
    tree_height_ = 5
    # 
    action_script = []
    for lon in range(len(DcpLonAction)): # 遍历每一个纵向动作 
        ongoing_action_seq = []
        ongoing_action_seq.append(
            DcpAction(DcpLonAction.by_index(lon), ongoing_action_.lat, ongoing_action_.t))

        for h in range(1, tree_height_):
            for lat in range(len(DcpLatAction)):
                if lat != ongoing_action_.lat:
                    actions = AppendActionSequence(
                        ongoing_action_seq,
                        DcpAction(DcpLonAction.by_index(lon), DcpLatAction.by_index(lat), 2.0),
                        tree_height_ - h)
                    action_script.append(actions)
            ongoing_action_seq.append(
                DcpAction(DcpLonAction.by_index(lon), ongoing_action_.lat, 2.0))
        action_script.append(ongoing_action_seq)

    # override the last layer time
    # for action_seq in action_script:
    #     action_seq[-1].t = last_layer_time_

    return action_script

def draw(script):
    # 给定一个action序列，画成节点图
    import matplotlib.pyplot as plt
    for action_seq in script:
        for action in action_seq:
            print(action)
        print("----")

pprint.pprint(build_tree())
