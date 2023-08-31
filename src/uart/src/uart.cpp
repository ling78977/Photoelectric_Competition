/***
 * @
 * @   ┏┓　　　┏┓
 * @ ┏┛┻━━━┛┻┓
 * @ ┃　　　　　　　┃
 * @ ┃　　　━　　　┃
 * @ ┃　＞　　　＜　┃
 * @ ┃　　　　　　　┃
 * @ ┃...　⌒　...　┃
 * @ ┃　　　　　　　┃
 * @ ┗━┓　　　┏━┛
 * @     ┃　　　┃　
 * @     ┃　　　┃
 * @     ┃　　　┃
 * @     ┃　　　┃  神兽保佑
 * @     ┃　　　┃  代码无bug　　
 * @     ┃　　　┃
 * @     ┃　　　┗━━━┓
 * @     ┃　　　　　　　┣┓
 * @     ┃　　　　　　　┏┛
 * @     ┗┓┓┏━┳┓┏┛
 * @       ┃┫┫　┃┫┫
 * @       ┗┻┛　┗┻┛
 * @
 * @Author: ling78977 1986830413@qq.com
 * @Date: 2023-07-18 18:21:51
 * @LastEditors: ling78977 1986830413@qq.com
 * @LastEditTime: 2023-08-29 02:13:50
 * @FilePath: /Photoelectric_Competition/src/uart/src/uart.cpp
 * @Description:
 * @
 * @Copyright (c) 2023 by ${git_name_email}, All Rights Reserved.
 */

#include "../include/uart.h"

namespace uart {

  inline unsigned char SerialPort::returnHighBit(const int &Byte) {
    exchangebyte_ = (Byte >> 8) & 0xff;

    return exchangebyte_;
  }
  inline unsigned char SerialPort::returnLowBit(const int &Byte) {
    exchangebyte_ = Byte & 0xff;

    return exchangebyte_;
  }

  inline int16_t SerialPort::mergeIntoBytes(const unsigned char &highbit,
    const unsigned char &lowbit) {
    exchangebit_ = (highbit << 8) | lowbit;

    return exchangebit_;
  }

  bool SerialPort::Isopened( ) {
    if (fd != -1) {
      return true;
    }
    else {
      return false;
    }
  }
  SerialPort::SerialPort(const std::string _serial_config) {
    cv::FileStorage serial(_serial_config, cv::FileStorage::READ);
    if (!serial.isOpened( )) {
      std::cerr << "\033[31mopen uart files failed !\033[0m" << std::endl;
    }
    serial ["PREFERRED_DEVICE"] >> serial_config_.preferred_device;
    serial ["SET_BAUDRATE"] >> serial_config_.set_baudrate;
    serial ["SHOW_SERIAL_INFORMATION"] >> serial_config_.show_serial_information;
    serial.release( );
    // std::string DeviceName = serial_config_.preferred_device.c_str();
    // struct termios newstate;
    // bzero(&newstate, sizeof(newstate));

    // fd = open(DeviceName.c_str(), O_RDWR | O_NOCTTY);  // 设置为阻塞模式
    // // fd = open(DeviceName.c_str(), O_RDWR );
    // if (fd == -1) {
    //   std::cerr << "Open serial device failed:" << DeviceName << std::endl;
    // } else {
    //   std::cout << "Open serial device success:" << DeviceName << std::endl;
    // }

    const char *DeviceName [] = { serial_config_.preferred_device.c_str( ),
                                "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3" };

    struct termios newstate;
    bzero(&newstate, sizeof(newstate));

    for (size_t i = 0; i != sizeof(DeviceName) / sizeof(char *); ++i) {
      fd = open(DeviceName [i], O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
      if (fd == -1) {
        std::cerr << "\033[31m open serial failed:" << DeviceName [i] << "\033[0m"
          << std::endl;
      }
      else {
        std::cout << "\033[1m open serial sucess:" << DeviceName [i] << "\033[0m"
          << std::endl;
        break;
      }
    }
    switch (serial_config_.set_baudrate) {
    case 1:
      cfsetospeed(&newstate, B115200);
      cfsetispeed(&newstate, B115200);
      break;
    case 10:
      // B921600
      cfsetospeed(&newstate, B921600);
      cfsetispeed(&newstate, B921600);
      break;
    default:
      cfsetospeed(&newstate, B115200);
      cfsetispeed(&newstate, B115200);
      break;
    }
    newstate.c_cflag |= CLOCAL | CREAD;
    newstate.c_cflag &= ~CSIZE;
    newstate.c_cflag &= ~CSTOPB;
    newstate.c_cflag |= CS8;
    newstate.c_cflag &= ~PARENB;

    newstate.c_cc [VTIME] = 0;
    newstate.c_cc [VMIN] = 0;

    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newstate);
    receivevision_.ivalue = 0;
  }

  SerialPort::~SerialPort(void) {
    if (!close(fd)) {
      std::cout << "\033[1mClose serial device success:" << fd << "\033[0m"
        << std::endl;
    }
  };

  int SerialPort::ReturnReceive( ) {
    int data = receivevision_.ivalue;
    receivevision_.ivalue = 0;
    return data;
  }
  void SerialPort::getDataForSend(const int &row, const int &col) {
    writerow_.ivalue = row;
    writecol_.ivalue = col;
    write_buff_ [0] = 0xff;
    write_buff_ [1] = writerow_.cvalue [0];
    write_buff_ [2] = writerow_.cvalue [1];
    write_buff_ [3] = writerow_.cvalue [2];
    write_buff_ [4] = writerow_.cvalue [3];
    write_buff_ [5] = writecol_.cvalue [0];
    write_buff_ [6] = writecol_.cvalue [1];
    write_buff_ [7] = writecol_.cvalue [2];
    write_buff_ [8] = writecol_.cvalue [3];
    write_buff_ [9] = 0xfe;
    std::cout << " \033[1mUart send data length is " << sizeof(write_buff_)
      << "\033[0m" << std::endl;
  }
  void SerialPort::writeData(const int &row, const int &col) {
    getDataForSend(row, col);
    write_message_ = write(fd, write_buff_, sizeof(write_buff_));
    // printf("write_message_:",write_message_);
    std::cout << "write_message_:" << write_message_ << std::endl;
    if (serial_config_.show_serial_information == 1) {
      if (write_message_ != -1) {
        std::cout << "writeData:\n"
          << "row:" << writerow_.ivalue << "\ncol:" << writecol_.ivalue
          << std::endl;
        std::cout << "writeData with 0x:" << std::endl;
        for (int i = 0; i != 10; i++) {
          printf("write_buff_[%ld]:%x\n", i, write_buff_ [i]);
        }
      }
      else {
        std::cout << "send data defaled!" << std::endl;
      }
    }
  }
  void SerialPort::writeData(std::vector<std::pair<uint8_t, uint8_t>> _keypoint) {
    int write_buff_length = 2 * (_keypoint.size( )) + 2;
    uint8_t write_buff [write_buff_length];
    write_buff [0] = 0xff;
    for (int i = 0; i < _keypoint.size( ); i++) {
      write_buff [1 + 2 * i] = _keypoint [i].first;
      write_buff [2 + 2 * i] = _keypoint [i].second;
    }
    write_buff [write_buff_length - 1] = 0xfe;
    write_message_ = write(fd, write_buff, sizeof(write_buff));
    std::cout << "write_message_-> " << write_message_ << std::endl;
    if (serial_config_.show_serial_information == 1) {
      if (write_message_ != -1) {
        std::cout
          << "<--------------------------writeData-------------------------->"
          << std::endl;
        std::cout << "writeData with 0x:" << std::endl;
        for (int i = 0; i != write_buff_length; i++) {
          printf("write_buff[%d]:%x\n", i, write_buff [i]);
        }
        std::cout << "writeData with row_and_col:" << std::endl;
        for (int i = 0; i < _keypoint.size( ); i++) {
          printf("\033[33mrow:%d col:%d\033[0m \n", (int) write_buff [1 + 2 * i],
            (int) write_buff [2 + 2 * i]);
        }
      }
      else {
        std::cout << "\033[31msend data defaled!\033[0m" << std::endl;
      }
    }
  }

  int SerialPort::receiveData1( ) {
    while (true) {
      memset(receive_buff_, '0', REC_INFO_LENGTH * 2);
      read_message_ = read(fd, receive_buff_temp_, sizeof(receive_buff_temp_));
      // std::cout << "read_message_:" << read_message_ << std::endl;
      for (size_t i = 0; i != sizeof(receive_buff_temp_); ++i) {
        // 判断帧头帧尾
        if (receive_buff_temp_ [i] == 0xff &&
          receive_buff_temp_ [i + sizeof(receive_buff_) - 1] == 0xfe) {
          // 使用串口打印标志
          if (serial_config_.show_serial_information == 1) {
            std::cout << "ReadDate-->" << std::endl;
            // 遍历接收数据（uint8_t）
            for (size_t j = 0; j != sizeof(receive_buff_); ++j) {
              // 转存数据到receive_buff_
              receive_buff_ [j] = receive_buff_temp_ [i + j];
              printf("receive_buff_[%d]:%x\n", j, receive_buff_ [j]);
            }
            printf("receive:%d", receivevision_.ivalue);
            receivevision_.cvalue [0] = receive_buff_ [1];
            receivevision_.cvalue [1] = receive_buff_ [2];
            receivevision_.cvalue [2] = receive_buff_ [3];
            receivevision_.cvalue [3] = receive_buff_ [4];
            printf("\n");
          }
          else {
            for (size_t j = 0; j != sizeof(receive_buff_); ++j) {
              receive_buff_ [j] = receive_buff_temp_ [i + j];
            }
            receivevision_.cvalue [0] = receive_buff_ [1];
            receivevision_.cvalue [1] = receive_buff_ [2];
            receivevision_.cvalue [2] = receive_buff_ [3];
            receivevision_.cvalue [3] = receive_buff_ [4];
          }
          break;
        }
      }
      if (receivevision_.ivalue == 1) {
        break;
      }
    }
    int ret = receivevision_.ivalue;
    receivevision_.ivalue = 0;
    return ret;
  }

  int SerialPort::receiveData2( ) {
    while (true) {
      memset(receive_buff_, '0', REC_INFO_LENGTH * 2);
      read_message_ = read(fd, receive_buff_temp_, sizeof(receive_buff_temp_));
      // std::cout << "read_message_:" << read_message_ << std::endl;
      for (size_t i = 0; i != sizeof(receive_buff_temp_); ++i) {
        // 判断帧头帧尾
        if (receive_buff_temp_ [i] == 0xff &&
          receive_buff_temp_ [i + sizeof(receive_buff_) - 1] == 0xfe) {
          // 使用串口打印标志
          if (serial_config_.show_serial_information == 1) {
            std::cout << "ReadDate-->" << std::endl;
            // 遍历接收数据（uint8_t）
            for (size_t j = 0; j != sizeof(receive_buff_); ++j) {
              // 转存数据到receive_buff_
              receive_buff_ [j] = receive_buff_temp_ [i + j];
              printf("receive_buff_[%d]:%x\n", j, receive_buff_ [j]);
            }
            printf("receive:%d", receivevision_.ivalue);
            receivevision_.cvalue [0] = receive_buff_ [1];
            receivevision_.cvalue [1] = receive_buff_ [2];
            receivevision_.cvalue [2] = receive_buff_ [3];
            receivevision_.cvalue [3] = receive_buff_ [4];
            printf("\n");
          }
          else {
            for (size_t j = 0; j != sizeof(receive_buff_); ++j) {
              receive_buff_ [j] = receive_buff_temp_ [i + j];
            }
            receivevision_.cvalue [0] = receive_buff_ [1];
            receivevision_.cvalue [1] = receive_buff_ [2];
            receivevision_.cvalue [2] = receive_buff_ [3];
            receivevision_.cvalue [3] = receive_buff_ [4];
          }
          break;
        }
      }
      if (receivevision_.ivalue == 2) {
        break;
      }
    }
    int ret = receivevision_.ivalue;
    receivevision_.ivalue = 0;
    return ret;
  }

}  // namespace uart