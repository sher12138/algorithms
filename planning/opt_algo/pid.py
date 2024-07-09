## 标准PID 控制器 

"""
P ：比例控制器
I ：积分控制器
D ：微分控制器
"""

import numpy as np
import time

class PID:
    def __init__(self,p,i,d,current_time = None) -> None:
        self.Kp = p 
        self.Ki = i 
        self.Kd = d 

        self.sample_time = 0.00 
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        self.reset()


    def pid_control(self, feedback_value,current_time = None):
        """
        u(t) = k_p e(t) + k_i \int e(t)dt + k_d \frac{de(t)}{dt}
        """

        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if(delta_time >= self.sample_time):
            self.PTerm = self.Kp * error 
            self.ITerm += error * delta_time 

            if(self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif self.ITerm > self.windup_guard:
                self.ITerm = self.windup_guard
            
            self.DTerm = 0.0 
            if delta_time >0:
                self.DTerm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error 

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def reset(self):
        self.SetPoint = 0.0
        self.ITerm = 0.0 
        self.DTerm = 0.0 
        self.PTerm = 0.0 
        self.last_error = 0.0

        self.int_error = 0.0 
        self.windup_guard = 20.0 

        self.output = 0.0 

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain
    
    def setKd(self, derivative_gain):
        self.Kd = derivative_gain
    
    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

    







if __name__ == "__main__":
    pid = PID(1.2,1.0,0.001)
    pid.SetPoint = 0.0
    pid.setSampleTime(0.01)

    END = 100
    feedback = 0.0 
    feedback_list = []
    time_list = [] 
    setpoint_list = [] 

    for i in range(1,END):
        pid.pid_control(feedback)
        output = pid.output
        if pid.SetPoint > 0:
            feedback += (output - (1/i))
        if i > 9:
            pid.SetPoint = 1.0
        time.sleep(0.01)

        feedback_list.append(feedback)
        setpoint_list.append(pid.SetPoint)
        time_list.append(i)

    import matplotlib.pyplot as plt
    plt.plot(time_list,feedback_list)
    plt.plot(time_list,setpoint_list)
    plt.xlabel('time(s)')
    plt.ylabel('value')
    plt.show()
