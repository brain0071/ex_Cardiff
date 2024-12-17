import numpy as np
import math

def slerp(quat1: np.ndarray, quat2: np.ndarray, t: float) -> np.ndarray:
    quat1 = np.array(quat1)
    quat2 = np.array(quat2)

    cos_theta = np.dot(quat1, quat2)
    if cos_theta < 0.0:
        quat1 = -quat1
        cos_theta = -cos_theta
    cos_theta = max(min(cos_theta, 1.0), -1.0)
    if cos_theta > 0.95:
        return (1 - t) * np.array(quat1) + t * np.array(quat2)
    theta = math.acos(cos_theta)
    sin_theta = math.sqrt(1.0 - cos_theta * cos_theta)
    factor0 = math.sin((1 - t) * theta) / sin_theta
    factor1 = math.sin(t * theta) / sin_theta
    return np.add(np.multiply(quat1, factor0), np.multiply(quat2, factor1))


def main():
    quat1 = np.array([0.90068, -0.003767, -0.0037303, -0.43444])
    quat2 = np.array([0.92692, 0.060084, -0.052188, 0.36672])
    print("\n---- 0.5 ----------")
    python_result = slerp(quat1, quat2, 0.5)
    cpp_result = np.array([0.998373, 0.0307646, -0.0305468, -0.0369938])
    print(python_result)
    print(cpp_result)
    print("\n---- 0.25 ----------")
    python_result = slerp(quat1, quat2, 0.25)
    cpp_result = np.array([0.970297, 0.0137941, -0.0175135, -0.240873])
    print(python_result)
    print(cpp_result)
    print("\n---- 0.75 ----------")
    python_result = slerp(quat1, quat2, 0.75)
    cpp_result = np.array([0.983704, 0.046418, -0.0422723, 0.16847])
    print(python_result)
    print(cpp_result)
    # 0.25
    #
    # 0.75
    #

if __name__ == '__main__':
    main()
