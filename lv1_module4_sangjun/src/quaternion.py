"""문제 3 — 쿼터니언 변환과 SLERP. (학생 작성용 템플릿)

규약
----
- 쿼터니언은 길이 4 배열 **(x, y, z, w)** 다. 모듈 ③ 의 `quaternion_from_axis_angle` 과
  SciPy `Rotation.as_quat()` 와 같은 순서다. w 가 스칼라(실수부)다.
- q 와 -q 는 같은 회전이다 (이중 덮개). 비교할 때는 부호를 무시하거나 |q . q_ref| 를 본다.
- 보간은 항상 **짧은 호**를 택한다: q0 . q1 < 0 이면 q1 의 부호를 뒤집고 시작한다.

SciPy 는 검산(비교) 용도로만 쓴다. 이 파일 안에서는 numpy 만 사용한다.
"""

from __future__ import annotations

import numpy as np

__all__ = ["matrix_to_quaternion", "quaternion_to_matrix", "slerp", "lerp_quat", "quat_angle"]


def matrix_to_quaternion(R) -> np.ndarray:
    """회전행렬 (3,3) -> 단위 쿼터니언 (x, y, z, w).

    권장 방법 (Shepperd): trace 가 양수이면 w 부터, 아니면 대각성분이 가장 큰 축부터 계산해
    0 으로 나누는 일을 피한다. 180도 회전(trace = -1)에서도 동작해야 한다.

        t = trace(R)
        t > 0        : s = 2 sqrt(1 + t);      w = s/4; x = (R21 - R12)/s; ...
        R00 이 최대  : s = 2 sqrt(1 + R00 - R11 - R22);  x = s/4; w = (R21 - R12)/s; ...
        (R11, R22 최대인 경우도 같은 꼴)

    반환값은 반드시 정규화하고, w >= 0 이 되도록 부호를 맞춘다 (비교가 편해진다).
    """
    # TODO: 문제 3-1
    R = np.asarray(R, dtype=float)
    if R.shape != (3, 3):
        raise ValueError("R 은 (3, 3) 회전행렬이어야 합니다.")
    trace = np.trace(R)
    if trace > 0.0:
        s = 2.0 * np.sqrt(1.0 + trace)
        q = np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                      (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q = np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                          (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q = np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                          (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q = np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                          0.25 * s, (R[1, 0] - R[0, 1]) / s])
    q /= np.linalg.norm(q)
    return q if q[3] >= 0.0 else -q


def quaternion_to_matrix(q) -> np.ndarray:
    """단위 쿼터니언 (x, y, z, w) -> 회전행렬 (3,3).

        R = [[1 - 2(y^2 + z^2),   2(xy - zw),        2(xz + yw)],
             [2(xy + zw),         1 - 2(x^2 + z^2),  2(yz - xw)],
             [2(xz - yw),         2(yz + xw),        1 - 2(x^2 + y^2)]]

    입력이 정확히 단위가 아닐 수 있으므로 먼저 정규화한다. q 와 -q 는 같은 R 을 준다.
    """
    # TODO: 문제 3-1
    q = np.asarray(q, dtype=float).reshape(-1)
    if q.shape != (4,):
        raise ValueError("q 는 (4,) 쿼터니언이어야 합니다.")
    q_norm = np.linalg.norm(q)
    if q_norm == 0.0:
        raise ValueError("영 쿼터니언은 회전을 나타내지 않습니다.")
    x, y, z, w = q / q_norm
    return np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                     [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                     [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


def quat_angle(q0, q1) -> float:
    """두 단위 쿼터니언이 나타내는 회전 사이의 각도 [rad], 0 <= angle <= pi.

        angle = 2 * arccos(|q0 . q1|)
    """
    # TODO: 문제 3-2 (slerp 안에서 재사용)
    a = np.asarray(q0, dtype=float).reshape(-1)
    b = np.asarray(q1, dtype=float).reshape(-1)
    if a.shape != (4,) or b.shape != (4,):
        raise ValueError("두 입력 모두 (4,) 쿼터니언이어야 합니다.")
    denom = np.linalg.norm(a) * np.linalg.norm(b)
    if denom == 0.0:
        raise ValueError("영 쿼터니언은 비교할 수 없습니다.")
    return float(2.0 * np.arccos(np.clip(abs(np.dot(a, b)) / denom, -1.0, 1.0)))


def slerp(q0, q1, t: float, eps: float = 1e-8) -> np.ndarray:
    """구면 선형 보간 (Spherical Linear intERPolation).

        d = q0 . q1                      (d < 0 이면 q1 = -q1, d = -d 로 짧은 호 선택)
        omega = arccos(d)
        q(t) = [sin((1-t) omega) q0 + sin(t omega) q1] / sin(omega)

    경계 상황
    - 두 자세가 거의 같아 d > 1 - eps 이면 sin(omega) ~ 0 이라 나눗셈이 불안정하다.
      이때는 선형 보간 후 정규화로 대체한다.
    - d 는 부동소수점 오차로 1 을 살짝 넘을 수 있으므로 clip 한다.

    반환값은 단위 쿼터니언이어야 한다. t = 0 이면 q0, t = 1 이면 (부호를 맞춘) q1.
    """
    # TODO: 문제 3-2 · 3-5
    q0 = np.asarray(q0, dtype=float).reshape(-1)
    q1 = np.asarray(q1, dtype=float).reshape(-1)
    if q0.shape != (4,) or q1.shape != (4,):
        raise ValueError("q0, q1 은 (4,)이어야 합니다.")
    if np.linalg.norm(q0) == 0.0 or np.linalg.norm(q1) == 0.0:
        raise ValueError("영 쿼터니언은 보간할 수 없습니다.")
    q0, q1 = q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1, d = -q1, -d
    d = np.clip(d, -1.0, 1.0)
    if d > 1.0 - eps:
        q = (1.0 - t) * q0 + t * q1
    else:
        omega = np.arccos(d)
        q = (np.sin((1.0 - t) * omega) * q0 + np.sin(t * omega) * q1) / np.sin(omega)
    return q / np.linalg.norm(q)


def lerp_quat(q0, q1, t: float, normalize: bool = False) -> np.ndarray:
    """성분별 단순 선형 보간 (비교용).

        q(t) = (1 - t) q0 + t q1          (q0 . q1 < 0 이면 q1 부호를 먼저 뒤집는다)

    normalize=False 이면 정규화하지 않은 값을 그대로 돌려준다 — 크기가 1 에서 얼마나
    벗어나는지 관찰하는 데 쓴다. normalize=True 이면 정규화한다 (NLERP).
    """
    # TODO: 문제 3-4
    q0 = np.asarray(q0, dtype=float).reshape(-1)
    q1 = np.asarray(q1, dtype=float).reshape(-1)
    if q0.shape != (4,) or q1.shape != (4,):
        raise ValueError("q0, q1 은 (4,)이어야 합니다.")
    if np.dot(q0, q1) < 0.0:
        q1 = -q1
    q = (1.0 - t) * q0 + t * q1
    if normalize:
        length = np.linalg.norm(q)
        if length == 0.0:
            raise ValueError("보간 결과가 영 쿼터니언입니다.")
        q = q / length
    return q
