"""
运动学模型
"""
import math 

sin = math.sin
cos = math.cos
tan = math.tan

"""
\dot{x}= v * cos(psi)
\dot{y}= v * sin(psi)
\dot{\phi} = v * tan(\sigma) / L 


x = x + v * cos(psi) * dt
y = y + v * sin(psi) * dt
psi = psi + v / L * delta * dt
v = v + a * dt


"""
class KinematicModel():
    def __init__(self,x,y,psi,v,L,dt=0.1) -> None:
        self.x = x 
        self.y = y 
        self.psi = psi 
        self.v = v 
        self.L = L 
        self.dt = dt 
    
    def update(self,delta,a):
        self.x = self.x + self.v * cos(self.psi) * self.dt 
        self.y = self.y + self.v * sin(self.psi) * self.dt
        self.psi = self.psi + self.v / self.L * delta * self.dt
        self.v = self.v + a * self.dt 

    def get_state(self):
        return self.x,self.y,self.psi,self.v
    
    def state_space(self,ref_delta,ref_yaw):
        A = [
            [1,0,-self.v * sin(ref_yaw) * self.dt],
            [0,1,self.v * cos(ref_yaw) * self.dt],
            [0,0,1]
        ]
        B = [
            [cos(ref_yaw) * self.dt,0],
            [sin(ref_yaw) * self.dt,0],
            [tan(ref_delta) / self.L * self.dt,self.v / (self.L * cos(ref_delta) ** 2) * self.dt]
        ]

        return A,B 
    

    
