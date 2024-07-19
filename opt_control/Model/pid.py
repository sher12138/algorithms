# This file is used to define the PID controller class
import reference_path
import kinimatic_model
import matplotlib.pyplot as plt 
class PID():
    def __init__(self,Kp,Ki,Kd,target,upper,lower) -> None:
        self.Kp=Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target
        self.upper = upper
        self.lower = lower

        self.error = 0.0 
        self.pre_error = 0.0 
        self.sum_error = 0.0 

    def set_target(self,target):
        self.target = target

    def set_k(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def set_bound(self,upper,lower):
        self.upper = upper
        self.lower = lower

    def cal_output(self,state):
        self.error = self.target - state 
        u = self.error * self.Kp + self.sum_error * self.Ki + (self.error - self.pre_error) * self.Kd
        if u > self.upper:
            u = self.upper
        elif u < self.lower:
            u = self.lower
        self.pre_error = self.error
        self.sum_error += self.error
        print("error >>> ",self.pre_error,self.sum_error)
        return u 

    def reset(self):
        self.error = 0.0
        self.pre_error = 0.0
        self.sum_error = 0.0


    def set_sum_error(self,sum_error):
        self.sum_error = sum_error

if __name__ == "__main__":
    
    model = kinimatic_model.KinematicModel(0,1,0,0,2.2,1)
    ref_path = reference_path.ReferencePath()
    pid = PID(1.1,0.5,0.2, 2.0 ,4,-4)

    # 纵向控制
    plt_ref = []
    plt_control = []
    for i in range(20):
        robot_state = model.get_state()
        print("state:",robot_state)
        error,k,yaw,min_index = ref_path.cal_track_error(robot_state)
        a = pid.cal_output(model.v)
        model.update(0.0,a)

        plt_ref.append(2.0)
        plt_control.append(model.v)

    
    plt.plot(plt_ref,label="ref")
    plt.plot(plt_control,label="control")
    # print(plt_ref,plt_control)
    plt.legend()
    plt.show()



