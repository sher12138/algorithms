import numpy as np 
EPS = 1e-4

class LQR():
    def __init__(self,n) -> None:
        
        self.N = n

    def calRicatti(self,A,B,Q,R):
        A = np.array(A)
        B = np.array(B)
        Q = np.array(Q)
        R = np.array(R)
        Qf = Q 
        P_old = Qf 
        # P_new = Q + A^t p A - A^T P B (R + B^T P B)^-1 B^T P A
        for i in range(self.N):
            P_new = Q + A.T @ P_old @ A - A.T @ P_old @ B @ np.linalg.inv(R + B.T @ P_old @ B) @ B.T @ P_old @ A
            if np.linalg.norm(P_new - P_old) < EPS:
                break 
            P_old = P_new
        return np.array(P_new)

    def LQRControl(self,robot_state,ref_path,s0,A,B,Q,R):
        A = np.array(A)
        B = np.array(B)
        Q = np.array(Q)
        R = np.array(R)
        print("状态量：",            robot_state[0] - ref_path[s0][0],
            robot_state[1],ref_path[s0][1],
            robot_state[2],ref_path[s0][2])
        X = np.array([
            robot_state[0] - ref_path[s0][0],
            robot_state[1] - ref_path[s0][1],
            robot_state[2] - ref_path[s0][2]
        ]).reshape((3,1))

        P = self.calRicatti(A,B,Q,R)
        # 最优控制率 K = (R + B^T P B)^-1 B^T P A
        K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
        print(K,X)
        u = - K @ X
        # print(u.shape)
        return u[1,0]
    
