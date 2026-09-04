/*

stop_distance.cpp — 로봇의 제동(정지) 거리 계산

물리: 바퀴와 바닥 사이 마찰이 유일한 제동력이라고 보면
  감속도 a = mu * g   (mu: 마찰계수, g: 중력가속도)
  운동에너지 (1/2)m v^2 이 마찰일 (mu m g d) 로 모두 소모되어 정지하므로
  d = v^2 / (2 * mu * g)

빌드: g++ -Wall -std=c++17 stop_distance.cpp -o stop_distance
실행: ./stop_distance <속도[m/s]> <마찰계수>
  인자를 안 주면 값을 직접 입력받는다.

g++ -Wall -Wextra -O2 -std=c++17 -o robot main.cpp
#   ─┬──────────  ─┬─  ─┬────────
#    │             │    └ 언어 표준 지정 (ROS2 Humble은 C++17)
#    │             └ 최적화 레벨 2 (배포용; 디버깅 때는 -g)
#    └ 모든 경고 켜기 — 경고는 미래의 버그 목록입니다

*/

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
