# 모듈 ③ 과제 REPORT — 로봇 좌표 변환 수학 라이브러리 구현
> 범위: 로봇 좌표계 선형대수 및 동차변환 라이브러리 (`src/` 및 `notebooks/`)

---

## 1. 개요 및 과제 목표
본 과제는 로봇공학에서 필수적인 3차원 공간에서의 위치와 자세 표현, 좌표계 간 변환 및 좌표 변환 체인(Coordinate Chain)을 위한 수학 라이브러리를 NumPy 기반으로 직접 구현하고 검증하는 과제입니다.

### 주요 목표
- 3차원 벡터 기본 연산(내적, 외적, 정사영, 반대칭행렬) 및 가우스 소거법 기반 행렬식·역행렬·rank 구현
- 축별 회전 행렬, 로드리게스 회전 공식, Gram-Schmidt 재직교화 및 회전행렬 판정 알고리즘 구현
- 4x4 동차변환 행렬($T$)의 고속 역변환 공식($T^{-1}$) 유도 및 배치(Batch) 연산 구현
- 점(Point, $w=1$)과 방향(Vector, $w=0$)의 변환 분리 및 정규방정식 기반 최소자승법(Least Squares) 구현
- TF2 축소판 좌표 변환 체인(`CoordinateChain`) 구현 및 고유값 분해 기반 회전축·회전각 복원

---

## 2. 모듈별 주요 구현 내용 및 수학적 근거

### (1) 벡터 연산 모듈 (`src/vectors.py`)
- **내적 및 유클리드 노름**: $\text{dot}(a, b) = \sum a_i b_i$, $\text{norm}(v) = \sqrt{\text{dot}(v, v)}$ 직접 구현.
- **정사영 및 수직 성분**:
  $$\text{proj}_b(a) = \frac{a \cdot b}{b \cdot b} b, \quad \text{rej}_b(a) = a - \text{proj}_b(a)$$
- **반대칭행렬 및 외적**: 3차원 벡터 $a$에 대해 $[a]_\times$ 반대칭행렬을 만들고, 외적을 $\text{cross}(a, b) = [a]_\times b$로 계산.
- **가우스 소거법**: 부분 피벗팅(Partial Pivoting)을 적용하여 전진 소거 및 후진 대입으로 $Ax=b$ 해결. 행 사다리꼴(Row Echelon Form)을 통해 rank 및 행렬식($\det(A)$) 계산.

### (2) 회전 행렬 모듈 (`src/rotation.py`)
- **축별 회전 행렬**: $R_x(\theta), R_y(\theta), R_z(\theta)$ 명시적 구현.
- **로드리게스 공식**: 임의 단위축 $k$와 회전각 $\theta$에 대해:
  $$R = I + \sin\theta [k]_\times + (1 - \cos\theta) [k]_\times^2$$
- **Gram-Schmidt 재직교화**: 수치 오차로 파손된 회전행렬의 열벡터를 순차적으로 직교 정규화하여 직교성($R^T R = I, \det(R) = 1$) 복원.
- **회전축 및 회전각 복원**:
  $$\text{tr}(R) = 1 + 2\cos\theta \implies \theta = \arccos\left(\frac{\text{tr}(R) - 1}{2}\right)$$
  비대칭 성분 $R - R^T = 2\sin\theta [k]_\times$를 이용하여 단위 회전축 $k$ 복원.

### (3) 동차변환 모듈 (`src/transform.py`)
- **동차변환 생성 및 해석적 역변환**:
  $$T = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix} \implies T^{-1} = \begin{bmatrix} R^T & -R^T t \\ 0 & 1 \end{bmatrix}$$
  일반 역행렬 연산($O(N^3)$) 대신 $R^T$ 전치 및 행렬-벡터 곱으로 고속 계산.
- **배치 역변환 (`inv_T_batch`)**: `np.swapaxes`와 `np.einsum("nij,nj->ni", R_T, t)`를 이용해 반복문 없이 $(N, 4, 4)$ 묶음을 일괄 역변환.
- **점/방향 변환 구별**: $w=1$일 때는 회전과 병진 모두 적용, $w=0$일 때는 병진 무시 및 길이 보존.
- **정규방정식 최소자승법**: $(A^T A)x = A^T b$를 세우고 `inverse_gauss_jordan`으로 $x$ 및 잔차 $r = b - Ax$ 계산.

### (4) 좌표 변환 체인 모듈 (`src/coordinate_chain.py`)
- **`CoordinateChain`**: 부모-자식 프레임 간 변환 등록 및 트리 탐색(`_path_to_root`)을 통해 임의 프레임 간 $T_{\text{target} \leftarrow \text{source}}$ 자동 산출.
- **기본 체인 설정 (`default_chain`)**: `base` $\to$ `link` $\to$ `camera` 변환 구현 및 카메라 좌표의 `base` 기준 변환 기능 수립.

---

## 3. 단위 테스트 검증 결과 (`pytest`)

프로젝트 루트에서 `pytest -v` 실행 결과, 모든 79개 테스트 케이스 100% 통과:

```text
============================= test session starts ==============================
platform linux -- Python 3.10.12, pytest-9.1.1, pluggy-1.6.0
rootdir: /home/pa31/physicalai-lv1-Sangjun/lv1_module3_sangjun

tests/test_rotation.py::test_columns_are_orthonormal ... PASSED [72/72]
tests/test_rotation.py::test_gram_schmidt_restores_orthogonality PASSED
tests/test_transform.py::test_inv_T_gives_identity PASSED
tests/test_transform.py::test_inv_T_matches_generic_inverse PASSED
tests/test_transform.py::test_point_and_direction_differ PASSED
tests/test_transform.py::test_transform_points_is_vectorized PASSED
tests/test_transform.py::test_roundtrip_through_inverse PASSED
tests/test_transform.py::test_least_squares_matches_lstsq PASSED

============================== 79 passed in 0.23s ==============================
```

---

## 4. 노트북 실행 검증 결과 (`notebooks/`)
1. `01_vectors.ipynb`: 내적, 정사영, 외적, 가우스 소거법 완벽 통과.
2. `02_rotation.ipynb`: 회전행렬 비가환성($R_x R_y \neq R_y R_x$), 회전 궤적 시각화 성공.
3. `03_reorthogonalize.ipynb`: 수치 누적 오차에 따른 직교성 붕괴 및 Gram-Schmidt 복원 검증.
4. `04_linear_system.ipynb`: 피벗팅 적용 유무에 따른 수치적 안정성 비교(병태 조건 문제 해결).
5. `05_transform.ipynb`: $T_1 T_2$ vs $T_2 T_1$ 합성 순서 차이 시각화, `inv_T` 연산 속도 우위 확인, 최소자승법 카메라 자세 추정 성공.
6. `06_chain.ipynb`: `base` $\to$ `link` $\to$ `camera` 100만 개 점군 대규모 변환 왕복 오차 기계 정밀도 수준($< 10^{-15}$) 달성.
