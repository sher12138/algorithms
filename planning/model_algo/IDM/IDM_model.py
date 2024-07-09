import math 
import matplotlib.pyplot as plt 
class State:
    def __init__(self,v,v_load,s,s_front) -> None:
        self.v = v 
        self.v_load = v_load 
        self.s_front = s_front 
        self.s = s 

class Param:
    kminmumGap = 2.0 
    kDesiredHeadwayTime = 1.5
    kAcceleration = 1.0
    kComfortableBrakingDeceleration = 1.5
    kVehicleLength = 5.0
    kDesiredSpeed = 30.0
    kDelta = 4.0


def get_idm_desired_acc(param,cur_state):
    s_star = param.kminmumGap + max(0.0, cur_state.v * param.kDesiredHeadwayTime + cur_state.v *(cur_state.v - cur_state.v_load) / 
                                    (2.0 * math.sqrt(param.kAcceleration * param.kComfortableBrakingDeceleration)))
    
    s_alpha = max(0.1, cur_state.s_front - cur_state.s - param.kVehicleLength)

    acc = param.kAcceleration *   \
    (1.0 - pow(cur_state.v / param.kDesiredSpeed, param.kDelta) - pow(s_star / s_alpha, 2.0))
    # print("acc:",acc,"s_star:",s_star,"s_alpha:",s_alpha)
    return acc 

def draw(ego_s,ego_v,ego_a,load_s,startp):
    import matplotlib.pyplot as plt
    plt.subplot(330 + startp)
    plt.plot(ego_s,'r',label='ego_s')
    plt.plot(load_s,'y',label='load_s')
    delta_s = [ego_s[i] - load_s[i] for i in range(len(ego_s))]
    plt.plot(delta_s,'b',label='delta_s')
    plt.legend()
    plt.subplot(333 + startp)
    plt.plot(ego_v,'g',label='ego_v')
    plt.legend()
    plt.subplot(336 + startp)
    plt.plot(ego_a,'b',label='ego_a')
    plt.legend()
    # plt.show()

def get_iidm_desired_acc(param:Param,cur_state:State):
    a_free = param.kAcceleration *(1-pow(cur_state.v / param.kDesiredSpeed, param.kDelta)) if cur_state.v <= param.kDesiredSpeed else -param.kComfortableBrakingDeceleration * (1- pow(param.kDesiredSpeed / cur_state.v, \
                                                                                             param.kAcceleration * param.kDelta / param.kComfortableBrakingDeceleration))

    s_alpha = max(0.1,cur_state.s_front - cur_state.s - param.kVehicleLength)

    z = (param.kminmumGap + max(0.1,cur_state.v * param.kDesiredHeadwayTime + cur_state.v * (cur_state.v - cur_state.v_load) / (2.0 * math.sqrt(param.kAcceleration * param.kComfortableBrakingDeceleration)))) / s_alpha

    a_out = 0 
    if cur_state.v <= param.kDesiredSpeed:
        if z >= 1.0:
            a_out = param.kAcceleration * (1.0 - pow(z,2))
        else:
            a_out = a_free * (1 - pow(z, 2.0 * param.kAcceleration / a_free))
    else:
        if z>=1.0:
            a_out = a_free + param.kAcceleration * (1.0 - pow(z,2))
        else:
            a_out = a_free  
    
    a_out  = max(min(param.kAcceleration,a_out), -param.kComfortableBrakingDeceleration)
    return a_out 


def get_acc_desired_acc(param,cur_state):
    acc = get_iidm_desired_acc(param,cur_state)
    ds = max(0.1, cur_state.s_front - cur_state.s)
    acc_cah = (cur_state.v * cur_state.v * -param.kComfortableBrakingDeceleration) / (cur_state.v_load * cur_state.v_load) - 2 * ds * -param.kComfortableBrakingDeceleration

    coolness = 0.99 
    if acc >= acc_cah:
        return acc 
    else:
        return (1 - coolness) * acc + coolness * (acc_cah - param.kComfortableBrakingDeceleration * math.tanh((acc - acc_cah) / -param.kComfortableBrakingDeceleration))

def trajectory_generation(param,cur_state,time,func, k ):
    ego_s = []
    ego_v = []
    load_s = []
    ego_a = []
    for i in range(int(time / 0.1)):
        acc = func(param,cur_state)
        cur_state.v = cur_state.v + acc * 0.1
        cur_state.s = cur_state.s + cur_state.v * 0.1
        cur_state.s_front = cur_state.s_front + cur_state.v_load * 0.1
        ego_s.append(cur_state.s)
        ego_v.append(cur_state.v)
        ego_a.append(acc)
        load_s.append(cur_state.s_front)
    draw(ego_s,ego_v,ego_a,load_s,1 +k )



## Test
param1 = Param()
## 极端靠近工况 
cur_state1 = State(35,20,20,50) # v,v_load,s_front
cur_state2 = State(35,20,20,50) # v,v_load,s_front
cur_state3 = State(35,20,20,50) # v,v_load,s_front

## 正常工况 
cur_state1 = State(40,30,20,500) # v,v_load,s_front
cur_state2 = State(40,30,20,500) # v,v_load,s_front
cur_state3 = State(40,30,20,500) # v,v_load,s_front

plt.figure()
trajectory_generation(param1,cur_state1,100,get_idm_desired_acc,0)
trajectory_generation(param1,cur_state2,100,get_iidm_desired_acc, 1)
trajectory_generation(param1,cur_state3,100,get_acc_desired_acc,2)
plt.show()
