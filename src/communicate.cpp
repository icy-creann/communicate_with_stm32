#include "communicate_with_STM32/my_usb.h"
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
    Communicate communicate;

    if (!communicate.connect()) {
        std::cerr << "Failed to connect to any ttyUSB device." << std::endl;
        return 1;
    }

    // 接收线程会在需要时启动
    while (true) {
        std::string data = communicate.receive_usb();
        if (!data.empty()) {
            std::cout << "Received_data: " << data << std::endl;
        }
        communicate.send_usb_string("Hello, STM32!");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    communicate.close();
    return 0;
}
