#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>
#include <dv_msgs/msg/control_command.hpp>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <thread>

// Packet struct must match the ESP32 layout exactly
#pragma pack(push, 1)
struct IMUPacket {
    uint8_t start_byte;   // 0xAA
    float ax, ay, az;
    float gx, gy, gz;
    uint8_t checksum;
};
#pragma pack(pop)

class SerialIMUNode : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    std::thread serial_thread_;
    std::atomic<bool> should_exit_;
    int serial_fd_;
    std::string port_name_;
    int baud_rate_;
    std::string frame_id_;
    
    // Statistics
    int packet_count_;
    std::chrono::steady_clock::time_point last_stats_time_;

public:
    SerialIMUNode() : Node("serial_imu_node"), serial_fd_(-1), packet_count_(0), should_exit_(false) {
        // Declare parameters
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("frame_id", "map");
        
        // Get parameters
        port_name_ = this->get_parameter("port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // Create publisher
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        
        // Initialize serial port
        if (!initializeSerialPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
            rclcpp::shutdown();
            return;
        }
        
        serial_thread_ = std::thread(&SerialIMUNode::serialReadLoop, this);
        
        last_stats_time_ = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "Serial IMU Node initialized");
        RCLCPP_INFO(this->get_logger(), "Port: %s, Baud: %d, Frame: %s", 
                   port_name_.c_str(), baud_rate_, frame_id_.c_str());
    }
    
    ~SerialIMUNode() {
        should_exit_ = true;
        if (serial_thread_.joinable()) {
            serial_thread_.join();
        }
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    bool initializeSerialPort() {
        serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port %s: %s", 
                        port_name_.c_str(), strerror(errno));
            return false;
        }

        struct termios tty{};
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        // Set baud rate
        speed_t speed;
        switch (baud_rate_) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unsupported baud rate: %d", baud_rate_);
                close(serial_fd_);
                serial_fd_ = -1;
                return false;
        }
        
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        tty.c_iflag &= ~IGNBRK;                         // disable break processing
        tty.c_lflag = 0;                                // no signaling chars, no echo, no canonical processing
        tty.c_oflag = 0;                                // no remapping, no delays
        tty.c_cc[VMIN] = 1;                             // read blocks until at least 1 char
        tty.c_cc[VTIME] = 1;                            // 0.1 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
        tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls
        tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        // Flush any existing data
        tcflush(serial_fd_, TCIOFLUSH);
        
        return true;
    }
    
    uint8_t calculateChecksum(uint8_t* data, size_t len) {
        uint8_t sum = 0;
        for (size_t i = 0; i < len; ++i)
            sum ^= data[i];
        return sum;
    }
    
    bool readFully(uint8_t* buffer, size_t length) {
        size_t total = 0;
        while (total < length) {
            ssize_t n = read(serial_fd_, buffer + total, length - total);
            if (n <= 0) return false;
            total += n;
        }
        return true;
    }
    
    void serialReadLoop() {
        IMUPacket packet;
        uint8_t buffer[2];
        
        RCLCPP_INFO(this->get_logger(), "Serial reading thread started");
        
        while (!should_exit_ && rclcpp::ok()) {
            // 1. Wait for start byte 
            if (read(serial_fd_, buffer, 2) <= 0) {
                continue;
            }
            
            if (buffer[0] != 0xAA) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Packet Not Received - got 0x%02X instead of 0xAA", buffer[0]);
                continue;
            }
            
            if (buffer[0] == 0xEE && buffer[1] == 0xFF) {
                continue;
            }
            
            // 2. Read the rest of the packet (25 bytes) 
            ((uint8_t*)&packet)[0] = buffer[0];
            
            if (!readFully(((uint8_t*)&packet) + 1, sizeof(packet) - 1)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Incomplete packet");
                continue;
            }
            
            // 3. Verify checksum - exactly like your working code
            uint8_t calculated = calculateChecksum(((uint8_t*)&packet) + 1, sizeof(packet) - 2);
            if (packet.checksum != calculated) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Checksum failed: expected 0x%02X, got 0x%02X", 
                                    calculated, packet.checksum);
                continue;
            }
            
            // 4. Create and publish IMU message
            publishIMUData(packet);

            RCLCPP_INFO(this->get_logger(), "Accel: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f)",
                        packet.ax, packet.ay, packet.az,
                        packet.gx, packet.gy, packet.gz);

            packet_count_++;
            if (packet_count_ % 100 == 0) {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_time_).count();
                double rate = 100000.0 / elapsed;
                RCLCPP_INFO(this->get_logger(), "Received 100 packets in %ld ms (%.1f Hz)", elapsed, rate);
                last_stats_time_ = now;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Serial reading thread stopped");
    }
    
    void publishIMUData(const IMUPacket& packet) {
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

        // Header
        imu_msg->header.stamp = this->get_clock()->now();
        imu_msg->header.frame_id = frame_id_;
        
        // Linear acceleration (convert from g to m/s²)
        imu_msg->linear_acceleration.y = packet.ay;
        imu_msg->linear_acceleration.z = packet.az;
        imu_msg->linear_acceleration.x = packet.ax;
        
        // Angular velocity (convert from degrees/sec to rad/sec)
        imu_msg->angular_velocity.x = packet.gx * M_PI / 180.0;
        imu_msg->angular_velocity.y = packet.gy * M_PI / 180.0;
        imu_msg->angular_velocity.z = packet.gz * M_PI / 180.0;
        
        // Set covariance matrices
        for (int i = 0; i < 9; ++i) {
            imu_msg->orientation_covariance[i] = -1.0;  // No orientation data
            imu_msg->angular_velocity_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
            imu_msg->linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
        }
        
        // Publish
        imu_publisher_->publish(std::move(imu_msg));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SerialIMUNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
