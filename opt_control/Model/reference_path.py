import numpy as np 
import math 

sin = math.sin
cos = math.cos
tan = math.tan

PI = 3.1415926

class RefTraj():
    def __init__(self) -> None:
        self.xref = []
        self.dref = []
        self.ind = -1 


# x,y,theta,k 
class ReferencePath():
    def __init__(self) -> None:
        
        self.ref_path = [] 
        self.ref_x = []
        self.ref_y = [] 

        for i in range(1000):
            self.ref_path.append([0,0,0,0])
            self.ref_path[i][0] = 0.1 * i
            self.ref_path[i][1] = 2 * sin(self.ref_path[i][0] / 3.0)

            self.ref_x.append(self.ref_path[i][0])
            self.ref_y.append(self.ref_path[i][1])

        for i in range(len(self.ref_path)):
            if i == 0:
                dx = self.ref_path[i+1][0] - self.ref_path[i][0]
                dy = self.ref_path[i+1][1] - self.ref_path[i][1]
                ddx = self.ref_path[i+2][0] - 2 * self.ref_path[i+1][0] + self.ref_path[i][0]
                ddy = self.ref_path[i+2][1] - 2 * self.ref_path[i+1][1] + self.ref_path[i][1]
            elif i == len(self.ref_path) - 1:
                dx = self.ref_path[i][0] - self.ref_path[i-1][0]
                dy = self.ref_path[i][1] - self.ref_path[i-1][1]
                ddx = self.ref_path[i][0] - 2 * self.ref_path[i-1][0] + self.ref_path[i-2][0]
                ddy = self.ref_path[i][1] - 2 * self.ref_path[i-1][1] + self.ref_path[i-2][1]
            else:
                dx = self.ref_path[i+1][0] - self.ref_path[i-1][0]
                dy = self.ref_path[i+1][1] - self.ref_path[i-1][1]
                ddx = self.ref_path[i+1][0] - 2 * self.ref_path[i][0] + self.ref_path[i-1][0]
                ddy = self.ref_path[i+1][1] - 2 * self.ref_path[i][1] + self.ref_path[i-1][1]
        
            self.ref_path[i][2] = math.atan2(dy,dx)
            # 计算曲率
            self.ref_path[i][3] = (ddy * dx - ddx * dy) / (dx ** 2 + dy ** 2) ** 1.5

    def cal_track_error(self,robot_state):
        x = robot_state[0]
        y = robot_state[1]
        d_x = []
        d_y = []
        d = []
        for i in range(len(self.ref_path)):
            d_x.append(self.ref_x[i]-x)
            d_y.append(self.ref_y[i] -y)
            d.append((d_x[i] ** 2 + d_y[i] ** 2)**0.5)
        # 
        min_d = min(d)
        min_index = d.index(min_d)
        # 计算横向误差
        yaw = self.ref_path[min_index][2]
        k = self.ref_path[min_index][3]
        angle = self.normalize_angle(yaw - math.atan2(d_y[min_index],d_x[min_index]))
        error = d[min_index] 

        if angle <0:
            error = -error
        return error,k,yaw,min_index


    def normalize_angle(self,angle):
        while angle > PI:
            angle -= 2 * PI
        while angle < -PI:
            angle += 2 * PI
        return angle
    
    def cal_ref_trajectory(self,robot_state,param,dl = 1.0):
        e,k,ref_yaw,ind = self.cal_track_error(robot_state)
        ref_traj = RefTraj()
        ref_traj.xref = [[] for i in param.Nx]
        ref_traj.dref = [[] for i in param.Nu]

        ref_traj.xref[0].append(self.ref_path[ind][0])
        ref_traj.xref[1].append(self.ref_path[ind][1])
        ref_traj.xref[2].append(self.ref_path[ind][2])

        ref_delta = math.atan2(param.L * k,1)

        for i in range(param.T):
            ref_traj.dref[0].append(robot_state[3])
            ref_traj.dref[1].append(ref_delta)

        travel = 0.0
        for i in range(param.T + 1):
            travel += abs(robot_state[3]) * param.dt
            dind = int(round(travel / dl))

            if ind + dind < self.ref_path.size():
                ref_traj.xref[0].append(self.ref_path[ind + dind][0])
                ref_traj.xref[1].append(self.ref_path[ind + dind][1])
                ref_traj.xref[2].append(self.ref_path[ind + dind][2])
            else:
                ref_traj.xref[0].append(self.ref_path[-1][0])
                ref_traj.xref[1].append(self.ref_path[-1][1])
                ref_traj.xref[2].append(self.ref_path[-1][2])
        
        return ref_traj

        


