#include "communicate_with_STM32/my_usb.h"
#include <thread>
#include <mutex>
#include <condition_variable>




Communicate::Communicate() : serial_connected(false), sp_ptr(nullptr), bytes_read(0) {
    std::cout << "Communicate start" << std::endl;
}

std::vector<std::string> Communicate::scanSerialDevices() {
    std::vector<std::string> devices;
    // 扫描ttyUSB设备
    for (int i = 0; i < 10; i++) {
        std::string port = "/dev/ttyUSB" + std::to_string(i);
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd != -1) {
            devices.push_back(port);
            ::close(fd);
        }
    }
    return devices;
}

void Communicate::scan_usb() {
    devices = scanSerialDevices();
    std::cout << "Scanning for available ttyUSB devices..." << std::endl;
    if (devices.empty()) {
        std::cout << "No ttyUSB devices found" << std::endl;
        return;
    }
    std::cout << "Found " << devices.size() << " ttyUSB device(s):" << std::endl;
    for (const auto& device : devices) {
        std::cout << device << std::endl;
    }
}

bool Communicate::connect(const std::string& port, int baudrate) {
    try {
        if (sp_ptr) {
            delete sp_ptr;
            sp_ptr = nullptr;
        }
        sp_ptr = new serial_port(iosev, port);
        // 设置串口参数
        sp_ptr->set_option(serial_port_base::baud_rate(baudrate));
        sp_ptr->set_option(serial_port_base::character_size(8));
        sp_ptr->set_option(serial_port_base::parity(serial_port_base::parity::none));
        sp_ptr->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        sp_ptr->set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
        serial_connected = true;
        std::cout << "Connected to " << port << std::endl;
        return true;
    } catch (const boost::system::system_error& e) {
        std::cout << "Failed to connect to " << port << ": " << e.what() << std::endl;
        serial_connected = false;
        return false;
    }
}

bool Communicate::connect() {
    // 扫描可用的ttyUSB设备
    devices = scanSerialDevices();
    if (devices.empty()) {
        std::cout << "No ttyUSB devices found" << std::endl;
        return false;
    }
    std::cout << "Scanning for available ttyUSB devices..." << std::endl;
    std::cout << "Found " << devices.size() << " ttyUSB device(s):" << std::endl;
    for (const auto& device : devices) {
        std::cout << device << std::endl;
    }
    // 尝试连接每个设备
    for (const auto& device : devices) {
        std::cout << "Trying to connect to " << device << "..." << std::endl;
        if (connect(device, 115200)) {
            return true;
        }
    }
    std::cout << "Could not connect to any ttyUSB port" << std::endl;
    return false;
}

bool Communicate::is_open() const {
    return serial_connected && sp_ptr && sp_ptr->is_open();
}

void Communicate::close() {
    if (sp_ptr) {
        try {
            sp_ptr->close();
            delete sp_ptr;
            sp_ptr = nullptr;
        } catch (...) {
        }
    }
    serial_connected = false;
    std::cout << "Serial port closed" << std::endl;
}

bool Communicate::send_usb_string(const std::string& data) {
    if (!is_open()) {
        return false;
    }
    try {
        // 构建消息队列，添加起始字节和结束字节
        std::vector<uint8_t> msg_queue;
        msg_queue.push_back(0xAA); // 起始字节
        msg_queue.insert(msg_queue.end(), data.begin(), data.end());
        msg_queue.push_back(0xBB); // 结束字节
        // 发送数据
        boost::system::error_code ec;
        write(*sp_ptr, buffer(msg_queue), ec);
        if (ec) {
            std::cout << "Error sending data: " << ec.message() << std::endl;
            return false;
        }
        std::cout << "Sent: " << data << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cout << "Error sending data: " << e.what() << std::endl;
        return false;
    }
}

// 接收线程相关（定义为 Communicate 类的静态成员）
std::thread Communicate::receive_thread;
std::atomic<bool> Communicate::receive_data_running(false);
std::string Communicate::receive_data_val;
std::mutex Communicate::receive_mutex;
std::condition_variable Communicate::receive_cv;

void Communicate::receive_usb_main() {
    while (receive_data_running) {
        if (!is_open()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        try {
            std::vector<uint8_t> msg_queue;
            uint8_t byte;
            // 等待起始字节
            while (receive_data_running) {
                boost::system::error_code ec;
                size_t n = read(*sp_ptr, buffer(&byte, 1), ec);
                if (ec || n == 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                if (byte == 0xAA) { // 起始字节
                    msg_queue.push_back(byte);
                    break;
                }
            }
            // 读取数据直到结束字节
            while (receive_data_running) {
                boost::system::error_code ec;
                size_t n = read(*sp_ptr, buffer(&byte, 1), ec);
                if (ec || n == 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                msg_queue.push_back(byte);
                if (byte == 0xBB) { // 结束字节
                    break;
                }
            }
            // 提取数据
            if (msg_queue.size() > 2) {
                std::string data(msg_queue.begin() + 1, msg_queue.end() - 1);
                std::cout << "Received: " << data << std::endl;
                {
                    std::lock_guard<std::mutex> lock(receive_mutex);
                    receive_data_val = data;
                }
                receive_cv.notify_one();
            }
        } catch (const std::exception& e) {
            std::cout << "Error receiving data: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

std::string Communicate::receive_usb() {
    std::string data;
    {
        std::lock_guard<std::mutex> lock(receive_mutex);
        data = receive_data_val;
        receive_data_val.clear();
    }
    if (!receive_data_running) {
        receive_data_running = true;
        receive_thread = std::thread(&Communicate::receive_usb_main, this);
        receive_thread.detach();
    }
    return data;
}

// 其他方法的实现
bool Communicate::send_usb(const float* data, size_t data_size) {
    if (!is_open()) {
        return false;
    }
    try {
        boost::system::error_code ec;
        write(*sp_ptr, buffer(data, data_size * sizeof(float)), ec);
        return !ec;
    } catch (...) {
        return false;
    }
}

float Communicate::getArrayElement(int index) const {
    if (index >= 0 && index < result.size()) {
        return result[index];
    }
    return 0.0f;
}

uint8_t Communicate::get_rxData(int index) const {
    if (index >= 0 && index < 108) {
        return rxData[index];
    }
    return 0;
}

void Communicate::importData(const float inputData[8]) {
    result.clear();
    for (int i = 0; i < 8; i++) {
        result.push_back(inputData[i]);
    }
}

void Communicate::async_send_usb(const float* data, size_t size) {
    if (!is_open()) {
        return;
    }
    sp_ptr->async_write_some(buffer(data, size * sizeof(float)),
        boost::bind(&Communicate::handle_error, this, boost::asio::placeholders::error));
}

bool Communicate::check_write_ready() const {
    return is_open();
}

void Communicate::handle_error(const boost::system::error_code& ec) {
    if (ec) {
        std::cout << "Error: " << ec.message() << std::endl;
    }
}

std::string Communicate::get_rx_string(size_t len) const {
    if (len == 0) {
        len = bytes_read;
    }
    return std::string(reinterpret_cast<const char*>(rxData), std::min(len, bytes_read));
}