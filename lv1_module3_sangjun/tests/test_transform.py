"""문제 5 — 동차변환 inv_T 검증 (pytest). [학생 작성용 템플릿]

지시문이 요구하는 것은 `inv_T` 검증이지만,
점/방향 구분과 벡터화, 최소자승까지 함께 검증해 두면 이후 문제에서 안전하다.

실행: 프로젝트 루트에서  pytest -v
"""

import numpy as np
import pytest

from src.rotation import rot_x, rot_y, rot_z
from src.transform import (
    inv_T,
    least_squares_normal_equation,
    make_T,
    transform_direction,
    transform_point,
    transform_points,
)


@pytest.fixture
def T():
    """테스트에 쓸 대표 동차변환 하나."""
    R = rot_z(0.9) @ rot_y(-0.35) @ rot_x(1.3)
    return make_T(R, [0.35, -0.15, 0.55])


def test_inv_T_gives_identity(T):
    inv_T_val = inv_T(T)
    assert np.allclose(inv_T_val @ T, np.eye(4)), "inv_T(T) @ T is not I"
    assert np.allclose(T @ inv_T_val, np.eye(4)), "T @ inv_T(T) is not I"


def test_inv_T_matches_generic_inverse(T):
    inv_T_val = inv_T(T)
    inv_np = np.linalg.inv(T)  # 검산용
    assert np.allclose(inv_T_val, inv_np), "inv_T(T) does not match np.linalg.inv(T)"


def test_point_and_direction_differ(T):
    v = np.array([1.0, 2.0, 3.0])
    p_trans = transform_point(T, v)
    d_trans = transform_direction(T, v)

    assert not np.allclose(p_trans, d_trans), "Point and direction transforms should differ"
    assert np.allclose(p_trans - d_trans, T[:3, 3]), "Difference should equal translation component"
    assert np.isclose(np.linalg.norm(d_trans), np.linalg.norm(v)), "Direction transform should preserve norm"


def test_transform_points_is_vectorized(T):
    rng = np.random.default_rng(42)
    P = rng.uniform(-1.0, 1.0, size=(10, 3))
    P_vectorized = transform_points(T, P, w=1.0)
    P_loop = np.array([transform_point(T, p) for p in P])
    assert np.allclose(P_vectorized, P_loop), "Vectorized transform_points does not match loop"


def test_roundtrip_through_inverse(T):
    rng = np.random.default_rng(42)
    P = rng.uniform(-1.0, 1.0, size=(10, 3))
    P_trans = transform_points(T, P, w=1.0)
    P_back = transform_points(inv_T(T), P_trans, w=1.0)
    assert np.allclose(P, P_back), "Roundtrip through inverse failed"


def test_least_squares_matches_lstsq():
    rng = np.random.default_rng(42)
    A = rng.uniform(-1.0, 1.0, size=(10, 3))
    x_true = np.array([1.5, -2.0, 0.5])
    noise = rng.normal(0, 0.01, size=(10,))
    b = A @ x_true + noise

    x_ls, residual = least_squares_normal_equation(A, b)
    x_lstsq, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

    assert np.allclose(x_ls, x_lstsq), "least_squares_normal_equation result does not match np.linalg.lstsq"
    assert np.allclose(A.T @ residual, np.zeros(3), atol=1e-10), "Residual is not orthogonal to column space of A"

