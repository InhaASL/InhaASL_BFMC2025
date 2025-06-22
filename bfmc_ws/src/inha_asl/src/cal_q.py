import numpy as np
from tf.transformations import quaternion_multiply, quaternion_inverse

# ① 방금 측정한 숫자를 그대로 넣는다 (w, x, y, z 순서 유지!)
q0      = np.array([0.3949798335, -0.7829301167,  0.4364952448, -0.2012045344])
q_roll  = np.array([0.0927287897, -0.8336570556, -0.2483492605, -0.4844996698])
q_pitch = np.array([0.8095457808, -0.3162327936,  0.1918127120, -0.4558950894])
q_yaw   = np.array([0.5607674699, -0.7625638935,  0.0233089369,  0.3217030409])

def diff(q2,q1):
    return quaternion_multiply(q2, quaternion_inverse(q1))

# ② 90° 단위 회전→기준 간 차이 쿼터니언
dq_r = diff(q_roll , q0)[1:]   # x,y,z 부분만
dq_p = diff(q_pitch, q0)[1:]
dq_y = diff(q_yaw , q0)[1:]

# ③ 3×3 축-매핑 행렬
M = np.vstack([dq_r, dq_p, dq_y]).T

# ④ 행렬 → 쿼터니언 (ROS 코드에 쓸 보정값)
w = np.sqrt(1 + np.trace(M)) / 2.0
x = (M[2,1] - M[1,2]) / (4*w)
y = (M[0,2] - M[2,0]) / (4*w)
z = (M[1,0] - M[0,1]) / (4*w)
print("q_offset  (x y z w) =", x, y, z, w)
