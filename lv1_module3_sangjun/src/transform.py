"""문제 5 — 4x4 동차변환 모듈. (학생 작성용 템플릿)

동차변환 생성/역변환, 점과 방향의 구분, 벡터화된 점군 변환,
정규방정식 기반 최소자승법을 직접 구현한다.
"""

from __future__ import annotations

import numpy as np

from .vectors import inverse_gauss_jordan

__all__ = [
    "make_T",
    "inv_T",
    "inv_T_batch",
    "to_homogeneous",
    "transform_point",
    "transform_direction",
    "transform_points",
    "least_squares_normal_equation",
    "rmse",
]


def make_T(R, t) -> np.ndarray:
    """회전 R(3x3)과 병진 t(3,)로 4x4 동차변환을 만든다.

        T = [[R, t],
             [0, 1]]

    R 이 3x3 이 아니면 ValueError.
    """
    R = np.asarray(R, dtype=float)
    t = np.asarray(t, dtype=float).ravel()

    if R.shape != (3, 3):
        raise ValueError(f"R 은 3x3 행렬이어야 합니다. 받은 shape={R.shape}")
    if t.shape != (3,):
        raise ValueError(f"t 는 3차원 벡터이어야 합니다. 받은 shape={t.shape}")

    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inv_T(T) -> np.ndarray:
    """동차변환의 역변환. **일반 역행렬 함수를 쓰지 않고** 공식으로 구한다.

        T^-1 = [[R^T, -R^T t],
                [  0,      1]]

    유도: T^-1 을 [[S, u], [0, 1]] 로 두고 T T^-1 = I 를 풀면
          R S = I -> S = R^T (R 이 직교),  R u + t = 0 -> u = -R^T t.

    4x4 가 아니면 ValueError.
    """
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        raise ValueError(f"T 는 4x4 행렬이어야 합니다. 받은 shape={T.shape}")

    R = T[:3, :3]
    t = T[:3, 3]
    R_T = R.T

    T_inv = np.eye(4, dtype=float)
    T_inv[:3, :3] = R_T
    T_inv[:3, 3] = -R_T @ t
    return T_inv


def inv_T_batch(Ts) -> np.ndarray:
    """(N, 4, 4) 동차변환 묶음을 **반복문 없이** 한 번에 역변환한다."""
    Ts = np.asarray(Ts, dtype=float)
    if Ts.ndim != 3 or Ts.shape[1:] != (4, 4):
        raise ValueError(f"Ts 는 (N, 4, 4) 형상이어야 합니다. 받은 shape={Ts.shape}")

    R = Ts[:, :3, :3]
    t = Ts[:, :3, 3]
    R_T = np.swapaxes(R, 1, 2)
    neg_R_T_t = -np.einsum("nij,nj->ni", R_T, t)

    N = Ts.shape[0]
    Ts_inv = np.zeros((N, 4, 4), dtype=float)
    Ts_inv[:, :3, :3] = R_T
    Ts_inv[:, :3, 3] = neg_R_T_t
    Ts_inv[:, 3, 3] = 1.0
    return Ts_inv


def to_homogeneous(P, w: float = 1.0) -> np.ndarray:
    """(3,) 또는 (N,3) 좌표에 마지막 성분 w 를 붙인다.

    w = 1 이면 점(위치), w = 0 이면 방향(벡터).
    """
    P = np.asarray(P, dtype=float)
    if P.ndim == 1:
        if P.shape[0] != 3:
            raise ValueError(f"1차원 좌표는 3개 성분이어야 합니다. 받은 shape={P.shape}")
        return np.array([P[0], P[1], P[2], w], dtype=float)
    elif P.ndim == 2:
        if P.shape[1] != 3:
            raise ValueError(f"2차원 좌표는 (N, 3) 형상이어야 합니다. 받은 shape={P.shape}")
        w_col = np.full((P.shape[0], 1), w, dtype=float)
        return np.hstack([P, w_col])
    else:
        raise ValueError(f"1차원 또는 2차원 배열이어야 합니다. 받은 ndim={P.ndim}")


def transform_point(T, p) -> np.ndarray:
    """점 변환 (w = 1): 회전과 병진이 모두 적용된다. 반환은 (3,)."""
    T = np.asarray(T, dtype=float)
    p_h = to_homogeneous(p, w=1.0)
    return (T @ p_h)[:3]


def transform_direction(T, v) -> np.ndarray:
    """방향 변환 (w = 0): 회전만 적용되고 병진은 무시된다. 반환은 (3,)."""
    T = np.asarray(T, dtype=float)
    v_h = to_homogeneous(v, w=0.0)
    return (T @ v_h)[:3]


def transform_points(T, P, w: float = 1.0) -> np.ndarray:
    """(N,3) 점군을 **반복문 없이** 한 번에 변환한다. (3,) 입력도 받아야 한다."""
    T = np.asarray(T, dtype=float)
    P = np.asarray(P, dtype=float)
    if P.ndim == 1:
        P_h = to_homogeneous(P, w=w)
        return (T @ P_h)[:3]
    elif P.ndim == 2:
        P_h = to_homogeneous(P, w=w)
        res_h = P_h @ T.T
        return res_h[:, :3]
    else:
        raise ValueError(f"1차원 또는 2차원 배열이어야 합니다. 받은 ndim={P.ndim}")


def least_squares_normal_equation(A, b):
    """정규방정식 (A^T A) x = A^T b 를 직접 세워 최소자승해를 구한다.

    - (A^T A) 의 역행렬은 문제 4 에서 만든 `inverse_gauss_jordan` 으로 구한다
      (`np.linalg.lstsq` 는 노트북에서 **비교 대상**으로만 쓴다).
    - 근거: 잔차 r = b - A x 가 최소일 때 r 은 A 의 열공간에 수직이므로 A^T r = 0.

    Returns
    -------
    x : 최소자승해
    residual : b - A x
    """
    A = np.asarray(A, dtype=float)
    b = np.asarray(b, dtype=float)

    A_T_A = A.T @ A
    A_T_b = A.T @ b

    inv_A_T_A = inverse_gauss_jordan(A_T_A)
    x = inv_A_T_A @ A_T_b
    residual = b - A @ x
    return x, residual


def rmse(residual) -> float:
    """잔차의 RMSE = sqrt(mean(r^2))."""
    res = np.asarray(residual, dtype=float)
    return float(np.sqrt(np.mean(res ** 2)))

