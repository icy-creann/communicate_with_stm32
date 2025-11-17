#include "include/communicate_with_STM32/my_usb.h"


/*
    扫描串口设备
*/
std::vector<std::string> MYUART::scanSerialDevices() {
    std::vector<std::string> deviceList;
    std::regex re("^ttyUSB\\d+$");  // 匹配以 "ttyUSB" 开头并跟着数字的设备文件名

    namespace bfs = boost::filesystem;
    for (bfs::directory_iterator it("/dev"); it != bfs::directory_iterator(); ++it) {
        std::string filename = it->path().filename().string();                                                                                     
        
        // 使用正则表达式检查设备名是否符合 "ttyACM" 格式
        if (std::regex_match(filename, re)) {
            deviceList.push_back(it->path().string());
        }
    }

    return deviceList;
}


/*
    匹配串口设备   
*/
void MYUART::scan_usb()
{
    while (!serial_connected)
    {
        try
        {
            devices = scanSerialDevices();
            if (devices.empty()) 
            {
                std::cout << "未找到串口设备，正在重新扫描..." << std::endl;
                // 可以添加适当的延时，避免过于频繁地扫描，消耗过多资源
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            } 
            sp_ptr = new serial_port(iosev, devices[0]);
            // 设置波特率为115200
            sp_ptr->set_option(serial_port::baud_rate(115200));
            // 设置流控制为无（可以根据需要修改为硬件流控制或软件流控制）
            sp_ptr->set_option(serial_port::flow_control(serial_port::flow_control::none));
            // 设置奇偶校验为无（可以根据需要设置为奇校验或偶校验）
            sp_ptr->set_option(serial_port::parity(serial_port::parity::none));
            // 设置停止位为1位
            sp_ptr->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
            // 设置字符大小为8位
            sp_ptr->set_option(serial_port::character_size(8));
            serial_connected = true;
            std::cout << "串口已成功连接。" << std::endl;
        } 
        catch (const boost::system::system_error& e) 
        {
            std::cout << "无法打开串口: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}


/*
    接受串口数据   
*/
void MYUART::receive_usb()
{
        boost::system::error_code err;
        try 
        {
            bytes_read = sp_ptr->read_some(buffer(rxData, sizeof(rxData)), err);   
            if (err) //报错重新搜索串口
            {
                if (err == boost::asio::error::eof) {
                    std::cout << "串口连接已断开，尝试重新扫描串口设备..." << std::endl;
                    // 关闭当前串口
                    if (sp_ptr!= nullptr) {
                        sp_ptr->close();
                        delete sp_ptr;
                        sp_ptr = nullptr;
                    }
                    serial_connected = false;
                    // 进入重新扫描串口设备的循环
                    scan_usb();
                } else {
                    std::cout << "读取串口数据时出现其他错误: " << err.message() << std::endl;
                }
            } 
            else    
            {  
            }     
        } catch (const boost::system::system_error& e) {
            std::cout << "系统错误，读取串口数据时出现异常: " << e.what() << std::endl;     
        }
}


/*
    发送数据(float数组类型)
*/
bool MYUART::send_usb(const float* data, size_t data_size) {

    int index = 1;  // 用于跟踪 msg_queue 中的位置

    // 将输入的 float 数据依次复制到 msg_queue 中
    for (size_t i = 0; i < data_size; ++i) {
        memcpy(&msg_queue[index], &data[i], sizeof(float));
        index += sizeof(float);
    }

    // 串口写数据
    msg_queue[0] = 0xAA;
    msg_queue[1+data_size*sizeof(float)] = 0xBB;

    boost::system::error_code err;
    try 
    {
        sp_ptr->write_some(boost::asio::buffer(msg_queue, sizeof(msg_queue)), err);
        if (err) //报错重新搜索串口
        {
            if (err == boost::asio::error::eof) {
                std::cout << "串口连接已断开，尝试重新扫描串口设备..." << std::endl;
                // 关闭当前串口
                if (sp_ptr!= nullptr) {
                    sp_ptr->close();
                    delete sp_ptr;
                    sp_ptr = nullptr;
                }
                serial_connected = false;
                // 进入重新扫描串口设备的循环
                scan_usb();
            } else {
                std::cout << "串口连接已断开，尝试重新扫描串口设备..." << std::endl;
                // 关闭当前串口
                if (sp_ptr!= nullptr) {
                    sp_ptr->close();
                    delete sp_ptr;
                    sp_ptr = nullptr;
                }
                serial_connected = false;
                // 进入重新扫描串口设备的循环
                scan_usb();
            }
        } 
        else    
        { 
        }     
    } catch (const boost::system::system_error& e) {
        std::cout << "系统错误，发送串口数据时出现异常: " << e.what() << std::endl;     
    }

    return true;
}


/*
    发送数据(字符串类型)
*/
bool MYUART::send_usb_string(const std::string& msg) {
    boost::system::error_code err;
    try{
        sp_ptr->write_some(boost::asio::buffer(msg), err);
        if (err){
            if (err == boost::asio::error::eof) {
                std::cout << "串口连接已断开，尝试重新扫描串口设备..." << std::endl;
                // 关闭当前串口
                if (sp_ptr!= nullptr) {
                    sp_ptr->close();
                    delete sp_ptr;
                    sp_ptr = nullptr;
                }
                serial_connected = false;
                // 进入重新扫描串口设备的循环
                scan_usb();
            } else {
                std::cout << "串口连接已断开，尝试重新扫描串口设备..." << std::endl;
                // 关闭当前串口
                if (sp_ptr!= nullptr) {
                    sp_ptr->close();
                    delete sp_ptr;
                    sp_ptr = nullptr;
                }
                serial_connected = false;
                // 进入重新扫描串口设备的循环
                scan_usb();
            }
        }else{
        }
    } catch (const boost::system::system_error& e) {
        std::cout << "系统错误，发送串口数据时出现异常: " << e.what() << std::endl;     
    }
    return true;
}
 

/*
    异步发送数据
 */
void MYUART::async_send_usb(const float* data, size_t size) {
    if(sp_ptr && sp_ptr->is_open()) {
        boost::asio::async_write(*sp_ptr, 
            boost::asio::buffer(data, size*sizeof(float)),
            [this](const boost::system::error_code& ec, size_t) {
                if(ec) handle_error(ec);
            });
    }
}


/*
    获取数组元素
 */
float MYUART::getArrayElement(int index) const {
    if (index >= 0 && index < result.size()) {
        return result[index];
    }
    // 索引无效，返回 -1 作为错误标识
    return -1; 
}


/*
    获取接收数据数组元素
 */
uint8_t MYUART::get_rxData(int index) const {
    return rxData[index]; 
}


/*
    获取接收数据字符串
 */
std::string MYUART::get_rx_string(size_t len = 0) const {
    if (len == 0) len = bytes_read;
    return std::string(reinterpret_cast<const char*>(rxData), len);
}


/*
    获取接收数据长度
 */
size_t MYUART::get_bytes_read() const { 
    return bytes_read; 
}



/*
    导入外部数据的函数
*/
void MYUART::importData(const float inputData[8]) {
    for (int i = 0; i < 8; ++i) {
        txData[i] = inputData[i];
    }
}



/*
    添加发送状态检查
 */
bool MYUART::check_write_ready() const {
    return sp_ptr && sp_ptr->is_open();
}



/*
    错误处理函数
 */
void MYUART::handle_error(const boost::system::error_code& ec) {
    // 错误处理逻辑...
}



