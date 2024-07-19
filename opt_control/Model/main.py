import math
import opt_control.opt_algorithm.LQR as LQR 
import kinimatic_model
import matplotlib.pyplot as plt
import reference_path
import pid

if __name__ == "__main__":
    N = 500 
    target_speed = 2.0
    Q = [
        [3,0,0],
        [0,3,0],
        [0,0,3]
    ]
    R = [
        [2,0],
        [0,1]
    ]

    x_ = []
    y_ = [] 
    reference_p = reference_path.ReferencePath()
    model = kinimatic_model.KinematicModel(0,1,0,0,2.2,0.1)
    pid = pid.PID(1.1,0.5,0.2,2.0,4,-4)
    lqr = LQR.LQR(N)
    robot_state = [] 

    for i in range(700):
        robot_state = model.get_state()
        print("状态：",robot_state)
        error,k,yaw,min_index = reference_p.cal_track_error(robot_state)

        ref_delta = math.atan2(k * model.L,1)
        state_space = model.state_space(ref_delta,yaw)

        delta = lqr.LQRControl(robot_state,reference_p.ref_path,min_index,*state_space,Q,R)
        delta+=ref_delta

        u = pid.cal_output(model.v)
        model.update(delta,u)
        x_.append(robot_state[0])
        y_.append(robot_state[1])

    plt.plot(x_,y_,label="control point")
    plt.plot([rp[0] for rp in reference_p.ref_path], [rp[1] for rp in reference_p.ref_path])
    plt.legend()
    plt.show()

