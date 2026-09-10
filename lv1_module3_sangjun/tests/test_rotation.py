"""문제 3 — 회전 행렬의 수학적 성질 검증 (pytest). [학생 작성용 템플릿]

지시문이 요구하는 4가지를 각각 테스트 함수로 작성한다.

  1. 회전행렬의 열이 서로 직교하는 단위벡터인가   -> test_columns_are_orthonormal
  2. 행렬식이 1인가                               -> test_determinant_is_one
  3. 역행렬이 전치와 같은가                       -> test_inverse_equals_transpose
  4. 재직교화 결과가 직교행렬인가                 -> test_gram_schmidt_restores_orthogonality

작성 요령
--------
- `@pytest.mark.parametrize` 로 여러 축 x 여러 각도를 한 함수에서 검사하면
  테스트 하나가 여러 케이스를 담당한다 (아래 ANGLES / MAKERS 참고).
- 비교는 반드시 `np.isclose` / `np.allclose` 로 한다 (부동소수점).
- `np.linalg` 는 검산용으로만 쓰고, 쓸 때는 주석으로 검산임을 밝힌다.
- assert 에 실패 메시지를 붙이면 어디가 깨졌는지 바로 보인다.
- 4개는 **최소 개수**다. 반사 행렬 반례, 로드리게스 일치, 축·각 왕복 같은
  테스트를 더 붙이면 좋다.

실행: 프로젝트 루트에서  pytest -v
"""

import numpy as np
import pytest

from src.rotation import (
    axis_angle_from_matrix,
    gram_schmidt,
    is_rotation,
    orthogonality_error,
    rodrigues,
    rot_x,
    rot_y,
    rot_z,
)

ANGLES = [0.0, np.deg2rad(22.5), np.pi / 6, np.pi / 4, np.pi / 2, 2.0, np.pi, -1.234]
MAKERS = [rot_x, rot_y, rot_z]


@pytest.fixture
def rng():
    """난수는 반드시 시드를 고정한다."""
    return np.random.default_rng(42)


# --- 1. 열이 서로 직교하는 단위벡터인가 -------------------------------------

@pytest.mark.parametrize("maker", MAKERS)
@pytest.mark.parametrize("theta", ANGLES)
def test_columns_are_orthonormal(maker, theta):
    # TODO: 각 열의 길이가 1 인지, 서로 다른 두 열의 내적이 0 인지 검사
    R = maker(theta)

    for i in range(3):
        assert np.isclose(np.linalg.norm(R[:, i]), 1.0), f"Column {i} is not unit length"

    for i in range(3):
        for j in range(i + 1, 3):
            assert np.isclose(np.dot(R[:, i], R[:, j]), 0.0), f"Columns {i} and {j} are not orthogonal"

    #raise NotImplementedError("test_columns_are_orthonormal 을 작성하세요")


# --- 2. 행렬식이 1인가 --------------------------------------------------------

@pytest.mark.parametrize("maker", MAKERS)
@pytest.mark.parametrize("theta", ANGLES)
def test_determinant_is_one(maker, theta):
    # TODO: det(R) == 1 인지 검사
    R = maker(theta)
    det_R = np.linalg.det(R)  # 검산용
    assert np.isclose(det_R, 1.0), f"Determinant is not 1: det(R)={det_R}"
    #raise NotImplementedError("test_determinant_is_one 을 작성하세요")


# --- 3. 역행렬 == 전치 --------------------------------------------------------

@pytest.mark.parametrize("maker", MAKERS)
@pytest.mark.parametrize("theta", ANGLES)
def test_inverse_equals_transpose(maker, theta):
    # TODO: inv(R) == R.T 이고 R.T @ R == I 인지 검사

    R = maker(theta)
    R_inv = np.linalg.inv(R)  # 검산용
    R_T = R.T
    assert np.allclose(R_inv, R_T), f"Inverse is not equal to transpose: inv(R) != R.T"
    assert np.allclose(R_T @ R, np.eye(3)), f"Transpose times R is not identity: R.T @ R != I"
    # raise NotImplementedError("test_inverse_equals_transpose 를 작성하세요")


# --- 4. 재직교화 결과가 직교행렬인가 -----------------------------------------

def test_gram_schmidt_restores_orthogonality(rng):
    # TODO: 회전행렬에 작은 노이즈를 섞어 직교성을 깨뜨린 뒤,
    #       gram_schmidt 로 복구하면 직교성 오차가 기계정밀도 수준으로 줄고
    #       det 가 1 이며 is_rotation 이 True 인지 검사
    R = rot_x(0.3) @ rot_y(-0.4) @ rot_z(0.2)

    noise = rng.normal(scale=1e-3, size=R.shape)
    R_noisy = R + noise
    R_orthonormal = gram_schmidt(R_noisy)

    err_before = orthogonality_error(R_noisy)
    err_after = orthogonality_error(R_orthonormal)

    assert err_after < err_before, f"Orthogonality error did not decrease: before={err_before}, after={err_after}"
    assert np.allclose(orthogonality_error(R_orthonormal), 0.0), "Orthonormal matrix has non-zero orthogonality error"
    assert np.isclose(np.linalg.det(R_orthonormal), 1.0), f"Determinant is not 1: det(R)={np.linalg.det(R_orthonormal)}"
    assert is_rotation(R_orthonormal), "Result is not a valid rotation matrix"
    # raise NotImplementedError("test_gram_schmidt_restores_orthogonality 를 작성하세요")


# --- 여기부터는 추가 테스트 (권장) -------------------------------------------
# 예) def test_reflection_is_not_a_rotation():
#         """det = -1 인 반사 행렬은 직교여도 회전이 아니다."""
#
# 예) def test_rodrigues_matches_rot_z(theta): ...
# 예) def test_axis_angle_roundtrip(rng): ...
