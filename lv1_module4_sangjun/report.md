# 모듈 ④ 과제 최종 보고서 — 픽앤플레이스 로봇의 자세 추정과 궤적 생성

> 과제 폴더: `lv1_module4_sangjun`
> 작성일: 2026-09-11
> 프로젝트: physicalai-lv1-Sangjun

---

## 1. 과제 개요

이 과제는 모듈 ①~③에서 구축한 좌표계 변환, 회전, 변환, 보간, 자세 추정 도구를 통합하여, 점군 기반의 물체 자세를 추정하고 그 물체를 향해 로봇이 부드럽게 이동하는 end-to-end 파이프라인을 완성하는 것을 목표로 합니다.

핵심 흐름은 다음과 같습니다.

1. 카메라 좌표계에서 관측한 점군을 base 좌표계로 변환한다.
2. PCA, Kabsch, 평면 피팅을 활용해 물체 자세를 추정한다.
3. 추정된 목표 자세와 현재 자세 사이에서 SLERP와 보간 기법을 적용한다.
4. 위치/자세가 동시에 변하는 궤적을 생성하고 애니메이션으로 시연한다.
5. 최종 결과를 `presentation.md`, `demo.gif`, `requirements.txt` 등 제출물로 정리한다.

---

## 2. 구현된 내용

### 2-1. 좌표계 모델링과 파이프라인

- `notebooks/01_pipeline.ipynb` 에서 좌표계 체인을 정의하고, base / link / camera / object 간의 변환 구조를 시각화했습니다.
- `src/pose_pipeline.py` 에서 `PosePipeline` 클래스를 구현하였습니다.
- `PosePipeline` 은 다음 기능을 제공합니다.
  - `camera_to_base(P_cam)`
  - `base_to_camera(P_base)`
  - `object_pose_in_base(T_camera_object)`
  - `set_joint_angle(theta)`
- `T_base_link`, `T_link_camera`, `T_base_camera`, `T_camera_base` 를 계산하고, 관절 각도 변화에 따라 링크 변환이 반영되는 구조를 확인했습니다.

### 2-2. 쿼터니언과 SLERP

- `src/quaternion.py` 에서 회전행렬과 쿼터니언 간의 변환을 구현했습니다.
- `matrix_to_quaternion()`, `quaternion_to_matrix()`, `quat_angle()`, `slerp()`, `lerp_quat()` 를 구성하였습니다.
- SLERP는 두 자세 사이의 짧은 호를 따라 보간되도록 구현하였고, `q`와 `-q`가 같은 회전을 나타내는 점을 고려하였습니다.
- 경계 상황으로 거의 같은 자세와 정반대 자세를 처리하는 로직을 반영하였습니다.

### 2-3. 궤적 보간과 연속성 비교

- `src/trajectory.py` 에서 다음 기능을 구현했습니다.
  - `linear_interp()`
  - `cubic_spline_interp()`
  - `quintic_profile()`
  - `finite_diff()`
- 선형 보간, 큐빅 스플라인, 그리고 5차 다항식 프로파일의 차이를 비교해 보았습니다.
- 특히 `quintic_profile()` 은 시작/끝에서 속도와 가속도가 0이 되도록 설계되어, 로봇의 부드러운 운동 프로파일을 표현하는 데 적합합니다.

### 2-4. 자세 추정

- `src/pose_estimation.py` 에서 다음 함수를 구현했습니다.
  - `pca_axes()`
  - `kabsch()`
  - `fit_plane_lstsq()`
  - `remove_outliers()`
- 점군의 공분산을 이용해 주축을 추출하고, Kabsch 알고리즘으로 회전과 병진을 추정하였습니다.
- 이상치가 섞인 점군에 대해 평면 피팅과 잔차 기반의 이상치 제거를 적용하여 추정의 안정성을 높였습니다.

### 2-5. 시연과 발표 자료

- `notebooks/03_pose_estimation.ipynb` 에서 추정된 물체 자세를 목표 자세로 설정하고, 현재 자세에서 목표 자세까지의 궤적을 생성했습니다.
- 위치는 스플라인으로, 자세는 SLERP로 보간하여 물체 좌표계가 자연스럽게 이동하는 애니메이션을 구성하였습니다.
- `demo.gif` 가 생성되었고, `presentation.md` 에 파이프라인, 추정 결과, 오차, 한계, 개선 방향을 정리했습니다.
- `requirements.txt` 는 가상환경 기준으로 최신 상태로 반영되었습니다.

---

## 3. 산출물 구성

최종 제출물은 아래 구조로 정리되었습니다.

- `notebooks/01_pipeline.ipynb`
- `notebooks/02_interpolation.ipynb`
- `notebooks/03_pose_estimation.ipynb`
- `src/pose_pipeline.py`
- `src/quaternion.py`
- `src/trajectory.py`
- `src/pose_estimation.py`
- `src/vectors.py`
- `src/rotation.py`
- `src/transform.py`
- `src/coordinate_chain.py`
- `tests/test_pose_pipeline.py`
- `tests/test_quaternion.py`
- `tests/test_trajectory.py`
- `requirements.txt`
- `demo.gif`
- `presentation.md`
- `README.md`
- `과제4_픽앤플레이스_자세추정.md`

---

## 4. 최종 정리

이번 과제에서는 점군으로부터 물체의 자세를 추정하고, 그 자세를 목표로 이동할 수 있는 로봇의 전체 파이프라인을 구성하는 흐름을 완성하였습니다.

특히 주목할 점은 다음과 같습니다.

- 좌표계 체인과 camera-to-base 변환 파이프라인을 실제로 연결했다.
- 회전행렬과 쿼터니언의 변환을 구현하고 SLERP로 부드러운 자세 보간을 수행했다.
- 위치의 경우 선형/스플라인/5차 다항식 보간을 비교해 로봇 제어에 적합한 궤적을 설계했다.
- PCA와 Kabsch를 활용해 점군의 자세 추정을 수행했고, 평면 피팅과 이상치 제거를 통해 오차를 줄이는 방법까지 검토했다.
- 마지막으로 애니메이션과 발표 자료를 함께 정리하여 최종 제출물로 완성했다.

결론적으로, 현재 작업 폴더의 Module 4 최종 결과는 이미 완성된 상태이며, 본 보고서 파일은 최종 제출에 필요한 문서화 자료로 작성되었다.
