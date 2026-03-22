import numpy as np
from scipy.spatial.transform import Rotation as R

def degrees_to_quaternion(yaw, pitch, roll):
    """
    Degrees (Yaw, Pitch, Roll)를 Quaternion (x, y, z, w)으로 변환
    """
    # 'zyx' 순서는 고정축 기준(Extrinsic) 혹은 회전축 기준(Intrinsic)에 따라 달라질 수 있음
    # 일반적인 드론/모바일 로봇은 z(yaw) -> y(pitch) -> x(roll) 순서를 따름
    r = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
    return r.as_quat()  # [x, y, z, w] 형태로 반환

def quaternion_to_degrees(x, y, z, w):
    """
    Quaternion (x, y, z, w)를 Degrees (Yaw, Pitch, Roll)로 변환
    """
    r = R.from_quat([x, y, z, w])
    # 다시 degrees로 변환
    yaw_pitch_roll = r.as_euler('zyx', degrees=True)
    return yaw_pitch_roll  # [yaw, pitch, roll]

# 테스트
quat = degrees_to_quaternion(90, 0, 0)
print(f"Quaternion (x, y, z, w): {quat}")

deg = quaternion_to_degrees(*quat)
print(f"Degrees (Yaw, Pitch, Roll): {deg}")
