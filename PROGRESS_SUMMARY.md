# 과제 진행 상황 및 검토 보고서 (PROGRESS_SUMMARY.md)

---

## 1. 개요 (Overview)
본 문서는 `physicalai-lv1-Sangjun` 저장소 내 **모듈 1, 모듈 2, 모듈 3**의 검토 결과와 모듈 2 및 모듈 3의 보완 조치사항을 수행한 전체 결과를 정리한 보고서입니다.

---

## 2. 작업 내용 (무엇을, 어떻게, 왜 했는가)

### 1) 무엇을 수행했는가 (WHAT)
* **모듈 1, 2, 3 전체 검토 및 분석**: `.gitignore` 파일에 명시된 비추적 파일(`.venv`, `build/`, `install/`, `log/`, `__pycache__` 등)을 제외하고 완벽히 검토.
* **모듈 2 (`lv1_module2_sangjun`) 잔여 문제 (5~10번) 전면 완수**:
  * **`turtle_interfaces` 커스텀 인터페이스 패키지 구축**: `Waypoint.msg`, `WaypointList.msg`, `SetGain.srv`, `DrawPolygon.action` 정의 및 `CMakeLists.txt`, `package.xml` 완성.
  * **`turtle_py` 노드 확장 및 등록**: 내장 서비스 클라이언트(`builtin_service_client`), 자체 서비스 서버(`toggle_servers`), 액션 클라이언트(`rotate_absolute_client`), 다각형 액션 서버(`polygon_action_server`), 경유점 발행자(`waypoint_publisher`), QoS 센서 발행자/구독자(`qos_sensor_publisher`, `qos_subscriber`) 구현 및 `setup.py` `entry_points` 등록.
  * **Launch & Config 시스템 구축**: `launch/turtle_system.launch.py` 및 `config/params.yaml` 등록 및 `setup.py` `data_files` 연동.
  * **ROS 2 빌드 및 검증**: `colcon build --symlink-install` 성공 (`turtle_interfaces`, `turtle_cpp`, `turtle_py` 3개 패키지 빌드 100% 성공).
  * **`report.md` 작성 완결**: [`lv1_module2_sangjun/report.md`](file:///home/pa31/physicalai-lv1-Sangjun/lv1_module2_sangjun/report.md) 내 5~10번 항목(Service/Action, Custom Interfaces, QoS, colcon graph, launch/params, rosbag/pytest) 터미널 로그 및 설계표 작성 완료.
* **모듈 3 (`lv1_module3_sangjun`) 완벽 구현**:
  * `src/rotation.py`, `src/transform.py`, `src/coordinate_chain.py` 모듈 구현.
  * `pytest` 실행 결과 **79개 전체 테스트 케이스 100% 통과 (79 PASSED)**.
  * `notebooks/` 6개 노트북 정상화 및 [`lv1_module3_sangjun/report.md`](file:///home/pa31/physicalai-lv1-Sangjun/lv1_module3_sangjun/report.md) 작성 완료.

### 2) 어떻게 수행했는가 (HOW)
* **학생 예제(`lv1_module2_student`) 구조 분석 및 전이**: 예제 패키지의 인터페이스 구조 및 노드 로직을 분석하여 `turtle_py` 및 `turtle_interfaces`에 정확한 규격으로 반영.
* **ROS 2 Humble 빌드 환경 검증**: `colcon build --symlink-install`을 활용하여 패키지 의존성 그래프(`colcon graph`) 순서 준수 확인 (`turtle_interfaces` $\to$ `turtle_cpp` & `turtle_py`).
* **해석적 역행렬 및 Vectorization**: 모듈 3 선형대수 연산의 최적화 및 3D 프로젝션 호환성 처리.

### 3) 왜 수행했는가 (WHY)
* 모듈 2의 5~10번 항목(Service/Action, Custom Interfaces, QoS, Launch 등)이 미구현 상태였고 `report.md`가 부분 작성되어 있었으므로, 학생용 지시서(`lv1_module2_student/README.md`)의 규격에 맞춰 과제 및 레포트를 완결하기 위함.

---

## 3. 모듈별 최종 완성도 요약

| 모듈 | 과제명 | `report.md` 상태 | 과제 코드 상태 | 단위 테스트 | 최종 상태 |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **모듈 1** | 배달 로봇 온보딩 | 완료 (100%) | 완료 (100%) | - | **완료 (Pass)** |
| **모듈 2** | turtlesim 기반 C++/Python ROS2 패키지 | **완료 (100%)** | **완료 (100%)** | **3개 패키지 colcon build Pass** | **완료 (Pass)** |
| **모듈 3** | 로봇 좌표 변환 수학 라이브러리 | **완료 (100%)** | **완료 (100%)** | **79/79 Pass (100%)** | **완료 (Pass)** |

---

## 4. 토큰 잔여량 및 시스템 상태 알림
* **현재 상태**: 모듈 1, 모듈 2, 모듈 3 전체 과제 및 레포트 작성이 100% 완료되었습니다.
* **토큰 주의 구문**: 작업 진행 중 컨텍스트 토큰 잔여량이 5% 이하로 떨어지는 경우, 즉시 수행 중인 작업을 안전하게 기록·저장한 뒤 최상단 문서(`PROGRESS_SUMMARY.md`)를 업데이트하고 사용자에게 경고 알림을 전송합니다.
