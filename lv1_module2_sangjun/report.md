# 모듈 ② 과제 — turtlesim 기반 C++·Python ROS2 패키지 개발
> 범위: 7~16강 · 문제 10개 = 성취도 구현 항목 10개 (전 문제 루브릭 채점)

## 과제 소개 
> C++ 빌드 체계를 세우는 것에서 시작해, ROS2 가 기본으로 제공하는 turtlesim 을 로봇 대신 놓고 그 로봇과 대화하는 패키지를 밑바닥부터 만듭니다
> <br><br>
> 거북이의 자세(/turtle1/pose)를 읽어 상태를 발행하고, 속도 명령(/turtle1/cmd_vel)으로 움직이고, 내장 서비스·액션을 호출하고, 커스텀 인터페이스로 경유점을 주고받고, launch 로 한 번에 기동한 뒤 RViz2·rosbag·pytest 로 검증하는 순서입니다.
> <br><br>
> ros-humble-turtlesim 하나만 설치하면 노드가 실제로 동작하는지 화면에서 바로 보입니다.

## 참고 사항
>장애물 감지와 회피, 지도 작성, 경로 계획은 Lv.2 의 SLAM·Nav2 교과목에서 다룹니다. <br>
>이 과제에서 거북이를 움직이는 데 필요한 계산은 아래 두 줄이 전부입니다.

- 목표까지 거리 d = hypot(gx - x, gy - y)
- 목표를 향한 각도 θ = atan2(gy - y, gx - x) → 각속도는 k · normalize(θ - theta)


## 과제 목표 

- g++와 CMake로 다중 파일 C++ 프로그램을 빌드하고 컴파일·링크 에러를 진단·해결할 수 있어요.
- C++ 클래스·상속·가상함수와 스마트 포인터(RAII), STL 컨테이너로 센서 처리 코드를 작성할 수 있어요.
- rclpy 로 publisher/subscriber 노드를 작성하고 노드 생명주기를 다룰 수 있어요.
- rclcpp 로 C++ publisher/subscriber 노드를 작성할 수 있어요.
- Service/Action 서버-클라이언트로 노드 간 통신을 구성하고 피드백·취소를 처리할 수 있어요.
- 커스텀 인터페이스(.msg/.srv/.action)를 정의·빌드해 노드 간 통신에 사용할 수 있어요.
- QoS 프로파일을 설정하고 비호환으로 인한 통신 단절을 진단·해결할 수 있어요.
- colcon 워크스페이스와 launch 파일로 다중 노드 시스템을 빌드·기동하고 파라미터를 주입할 수 있어요.
- RViz2·rqt·rosbag 으로 시각화·기록·재생하고 pytest 로 단위 테스트를 작성할 수 있어요.
---

## 1. C++ 빌드 체계 세우기 — g++ 다중 파일 빌드와 CMake 전환

### 00. 로봇 제동거리 계산 
```cpp
// stop_distance.cpp

#include <cstdlib>
#include <iostream> 

namespace
{
    constexpr double kGravity = 9.81; 

    double stop_distance(double speed, double mu)
    {
        return (speed * speed) / (2.0 * mu * kGravity);
    }
} 

int main()
{
    double speed;
    double mu;

    std::cout << "속도(m/s)를 입력하세요: ";
    std::cin >> speed;

    std::cout << "마찰 계수를 입력하세요: ";
    std::cin >> mu;

    std::cout << "정지거리: "
              << stop_distance(speed, mu)
              << " m"
              << std::endl;

    return 0; 
}
```

```bash
# 터미널 출력 

pa31@pa31-Legion-Pro-5-16IAX10:~/KantPA/lv1_module2_sangjun/cpp_basic$ ./stop_distance
속도(m/s)를 입력하세요: 68
마찰 계수를 입력하세요: 5
정지거리: 47.1356 m
```

### 01. 모터 클래스 분리 및 수동 2단계 빌드 명령
#### 수동 2단계 빌드 명령어

```bash
# motor.cpp 와 main.cpp 빌드

pa31@pa31-Legion-Pro-5-16IAX10:~/KantPA/lv1_module2_sangjun/cpp_basic$ g++ -c motor.cpp -o motor.o
pa31@pa31-Legion-Pro-5-16IAX10:~/KantPA/lv1_module2_sangjun/cpp_basic$ g++ -c main.cpp  -o main.o
```
```bash
# motor.o 와 main.o 링크

pa31@pa31-Legion-Pro-5-16IAX10:~/KantPA/lv1_module2_sangjun/cpp_basic$ g++ motor.o main.o -o robot
```

### 02. undefined reference 에러 메시지 (출력) — 컴파일 에러와의 차이 설명
```bash
# main을 컴파일 하지 않은 상태에서 '-c' 없이 컴파일과 링크를 동시에 실행

pa31@pa31-Legion-Pro-5-16IAX10:~/KantPA/lv1_module2_sangjun/cpp_basic$ g++ -Wall -Wextra -O2 -std=c++17 motor.cpp -o motor.o
/usr/bin/ld: /usr/lib/gcc/x86_64-linux-gnu/11/../../../x86_64-linux-gnu/Scrt1.o: in function `_start':
(.text+0x1b): undefined reference to `main'
collect2: error: ld returned 1 exit status
```
> 링커가 실행프로그램을 만드려고 하는데 main() 을 못찾는 상황이다.

### 03. CMake 빌드 출력 (터미널 출력)

```bash
# CMake 빌드 출력

pa31@pa31-Legion-Pro-5-16IAX10:~/KantPA/lv1_module2_sangjun/cpp_basic/build$ ㅍcmake ..
-- Configuring done
-- Generating done
-- Build files have been written to: /home/pa31/KantPA/lv1_module2_sangjun/cpp_basic/build
pa31@pa31-Legion-Pro-5-16IAX10:~/KantPA/lv1_module2_sangjun/cpp_basic/build$ make
[ 20%] Building CXX object CMakeFiles/robot.dir/main.cpp.o
[ 40%] Building CXX object CMakeFiles/robot.dir/motor.cpp.o
[ 60%] Linking CXX executable robot
[ 60%] Built target robot
[ 80%] Building CXX object CMakeFiles/stop_distance.dir/stop_distance.cpp.o
[100%] Linking CXX executable stop_distance
[100%] Built target stop_distance
pa31@pa31-Legion-Pro-5-16IAX10:~/KantPA/lv1_module2_sangjun/cpp_basic/build$ 
```

### 04. 증분 빌드 시 재컴파일된 파일: motor.cpp — 판단 근거
#### 증분 빌드 출력
```bash
# motor.cpp 수정 수 make 실행 결과

pa31@pa31-Legion-Pro-5-16IAX10:~/KantPA/lv1_module2_sangjun/cpp_basic/build$ make
Consolidate compiler generated dependencies of target robot
[ 20%] Building CXX object CMakeFiles/robot.dir/motor.cpp.o
[ 40%] Linking CXX executable robot
[ 60%] Built target robot
Consolidate compiler generated dependencies of target stop_distance
[100%] Built target stop_distance
pa31@pa31-Legion-Pro-5-16IAX10:~/KantPA/lv1_module2_sangjun/cpp_basic/build$ 
```
#### 판단 근거 
>motor.cpp가 변경되었기 때문에 motor.cpp만 다시 컴파일되었다.<br>
>이후 변경된 motor.o를 반영하기 위해 robot을 다시 링크하였다.
>
>변경되지 않은 main.cpp와 stop_distance.cpp는 다시 컴파일되지 않았다.
>
>CMake는 이전 빌드 결과와 현재 소스 파일의 변경 여부 및 파일 간 의존성 정보를 바탕으로 다시 빌드가 필요한 대상을 판단한다.<br> 
>따라서 변경되지 않은 소스는 기존 빌드 결과를 재사용하고, 변경된 소스만 다시 컴파일한다.

## 2. 현대 C++로 센서 계층 구현 — RAII·다형성·STL

### 00. Sensor 클래스 정의 및 Lidar, IMU 구현
```cpp
#include <vector>
#include <iostream>
#include <memory>

class Sensor {
    public:
        virtual ~Sensor() = default;
        virtual std::vector<double> read() = 0;

        bool isConnected() const {
            return connected;
        }
    protected:
        bool connected = false;
};

class Lidar : public Sensor {
    public:
        Lidar() {
            connected = true; // Simulate that the sensor is connected
        }

        std::vector<double> read() override {
            // Simulate reading data from the Lidar sensor
            return {1.0, 2.0, 3.0}; // Example data
        }
};

class Imu : public Sensor {
    public:
        Imu() {
            connected = true; // Simulate that the sensor is connected
        }

        std::vector<double> read() override {
            // Simulate reading data from the IMU sensor
            return {0.1, 0.2, 0.3}; // Example data
        }
};


int main() {

    std::cout << "Program started!" << std::endl;

    std::vector<std::unique_ptr<Sensor>> sensors;

    sensors.push_back(std::make_unique<Lidar>());
    sensors.push_back(std::make_unique<Imu>());
    for (auto& s : sensors) {
        auto data = s->read();     // 다형성: 실제 타입의 read() 호출

        for (double value : data) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
    return 0;
}
```

### 01. 다형성 루프 출력
```bash
pa31@pa31-Legion-Pro-5-16IAX10:~/KantPA/lv1_module2_sangjun/cpp_basic/sensors$ g++ -Wall -std=c++17 sensor.cpp -o sensor
pa31@pa31-Legion-Pro-5-16IAX10:~/KantPA/lv1_module2_sangjun/cpp_basic/sensors$ ./sensor 
Program started!
1 2 3 
0.1 0.2 0.3 
```
> `std::vector<std::unique_ptr<Sensor>>` 에 Lidar와 Imu를 저장하고 virtual/override 기반의 다형성 루프를 구성하여 각 센서의 read()가 정상적으로 호출되는 것을 확인하였다.

### 02. 스택 객체와 힙 객체의 소멸 시점

기존 코드를 아래와 같이 수정하였다.
```cpp
//Sensor
public:
        virtual ~Sensor(){
            std::cout << "Sensor destructor called" << std::endl;
        }
        virtual std::vector<double> read() = 0;

```

```cpp
 Lidar() {
            connected = true; // Simulate that the sensor is connected
            std::cout << "Lidar constructor called" << std::endl;
            std::cout << "Lidar connected: " << std::boolalpha << connected << std::endl;
        }

        ~Lidar() {
            std::cout << "Lidar destructor called" << std::endl;
        }

        std::vector<double> read() override {
            // Simulate reading data from the Lidar sensor
            return {1.0, 2.0, 3.0}; // Example data
        }
```

```cpp
Imu() {
            connected = true; // Simulate that the sensor is connected
            std::cout << "Imu constructor called" << std::endl;
            std::cout << "Imu connected: " << std::boolalpha << connected << std::endl;
        }

        ~Imu() {
            std::cout << "Imu destructor called" << std::endl;
        }

        std::vector<double> read() override {
            // Simulate reading data from the IMU sensor
            return {0.1, 0.2, 0.3}; // Example data
        }
```

> 생성과 소멸 시점을 확인하기 위해 `Sensor`,` Lidar`, `Imu`의 생성자와 소멸자에 출력문을 추가한다.

다시 빌드하여 실행해보면 아래와 같은 출력이 나온다 
```bash
pa31@pa31-Legion-Pro-5-16IAX10:~/physicalai-lv1-Sangjun/lv1_module2_sangjun/cpp_basic/sensors$ ./sensor 
Program started!
Lidar constructor called
Lidar connected: true
Imu constructor called
Imu connected: true
1 2 3 
0.1 0.2 0.3 
Lidar destructor called
Sensor destructor called
Imu destructor called
Sensor destructor called
```

> `sensors`벡터에 `Lidar`와 `Imu` 객체가 각각 하나씩 저장되어 있으며, 프로그램이 종료될 때 두 객체가 소멸한다. 이때 각 객체의 소멸 과정에서 파생 클래스의 소멸자가 먼저 호출되고, 이후 부모 클래스인 `Sensor`의 소멸자가 호출된다. 따라서 `Sensor destructor called`가 두 번 출력된다.

### 03. 가상 소멸자를 뺏을 떄의 차이: 

### 04. count_if 결과: 0.5 이내 기록 ___ 개

### 05. 누수 검출 결과 → 수정 후 결과 (검출 도구 출력 비교)

## 3. rclpy 노드 작성 — 거북이 상태 발행자와 구독자


