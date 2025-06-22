import numpy as np
from tf.transformations import quaternion_multiply, quaternion_inverse

# 네 개를 입력해 주세요 (w, x, y, z 순)
q0      = np.array([w0,x0,y0,z0])
q_roll  = np.array([wr,xr,yr,zr])
q_pitch = np.array([wp,xp,yp,zp])
q_yaw   = np.array([wy,xy,yy,zy])

# ① 각 축 단위 회전(90°)이 카메라 frame에서 어떤 벡터로 나타나는지
dq_roll  = quaternion_multiply(q_roll , quaternion_inverse(q0))
dq_pitch = quaternion_multiply(q_pitch, quaternion_inverse(q0))
dq_yaw   = quaternion_multiply(q_yaw , quaternion_inverse(q0))

# ② 축 벡터( x,y,z 세 성분 )만 꺼냄
v_roll  = dq_roll[1:]
v_pitch = dq_pitch[1:]
v_yaw   = dq_yaw[1:]

# ③ “카메라 축 → ROS 축” 3×3 행렬을 구성
M_cam2ros = np.vstack([v_roll, v_pitch, v_yaw]).T
# 역행렬 = ROS→카메라 보정 행렬
M_ros2cam = np.linalg.inv(M_cam2ros)

# ④ 행렬 → quaternion
def mat2quat(M):
    w = np.sqrt(1+np.trace(M))*0.5
    x = np.sign(M[2,1]-M[1,2]) * np.sqrt(max(0,1+M[0,0]-M[1,1]-M[2,2]))*0.5
    y = np.sign(M[0,2]-M[2,0]) * np.sqrt(max(0,1-M[0,0]+M[1,1]-M[2,2]))*0.5
    z = np.sign(M[1,0]-M[0,1]) * np.sqrt(max(0,1-M[0,0]-M[1,1]+M[2,2]))*0.5
    return np.array([w,x,y,z])

q_offset = mat2quat(M_cam2ros)   # 바로 코드에 넣을 보정 quaternion
print("q_offset =", q_offset)
