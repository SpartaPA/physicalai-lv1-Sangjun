#include <vector>
#include <iostream>
#include <memory>

class Sensor {
    public:
        virtual ~Sensor(){
            std::cout << "Sensor destructor called" << std::endl;
        }
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
};

class Imu : public Sensor {
    public:
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
};


int main() {

    std::cout << "Program started!" << std::endl;

    {
        Lidar stackLidar;
        auto heapLidar = std::make_unique<Lidar>();

        std::cout << "Inside scope" << std::endl;
    }

    std::cout << "After scope" << std::endl;

    std::vector<std::unique_ptr<Sensor>> sensors;

    sensors.push_back(std::make_unique<Lidar>());
    sensors.push_back(std::make_unique<Imu>());
    for (const std::unique_ptr<Sensor>& s : sensors) {
        auto data = s->read();     // 다형성: 실제 타입의 read() 호출

        for (double value : data) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}