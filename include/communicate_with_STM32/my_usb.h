#ifndef CPP_MYUSB_H
#define CPP_MYUSB_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
 
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <regex>
#include <string>

using namespace std;        // IO 服务，用于处理异步操作
using namespace boost::asio;
using namespace boost::placeholders;

class MYUART{
public:
    std::vector<std::string> scanSerialDevices();
    void scan_usb();
    void receive_usb();
    bool send_usb(const float* data, size_t data_size); 
    bool send_usb_string(const std::string& data);
    float getArrayElement(int index) const;
    uint8_t get_rxData(int index) const;
    void importData(const float inputData[8]);
    void async_send_usb(const float* data, size_t size);
    bool check_write_ready()const;
    void handle_error(const boost::system::error_code& ec);
    std::string get_rx_string(size_t len = 0) const;
    size_t get_bytes_read() const { return bytes_read; }


private:
    bool serial_connected = false;
    std::vector<std::string> devices;
    io_service iosev;
    // 创建serial_port对象指针
    serial_port* sp_ptr = nullptr; 
    boost::system::error_code err;  
    //接受数据的长度
    size_t bytes_read;
    // 串口固定消息队列
    uint8_t msg_queue[14];
    // 串口固定接受缓冲
    uint8_t rxData[108];
    // 串口固定发送缓冲
    float txData[3];
    //中间数据
    std::vector<float> result;

};

#endif