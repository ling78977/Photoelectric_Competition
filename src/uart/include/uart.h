

#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <string>

namespace uart {
enum BufferLength {
  REC_INFO_LENGTH = 6,
  WRITE_BUFF_LENGTH = 10,
};
struct Serial_Config {
  std::string preferred_device;
  int set_baudrate = 115200;
  int show_serial_information = 1;
};
typedef union {  // 定义一个联合体，用于int数据与16进制的转换
  unsigned char cvalue[4];
  int ivalue;
} int_union;

class SerialPort {
 public:
  SerialPort(const std::string _serial_config);
  ~SerialPort();
  /**
   * @brief 返回高八位数据
   *
   * @param Byte
   * @return unsigned char
   */
  inline unsigned char returnHighBit(const int& Byte);

  /**
   * @brief 返回低八位数据
   *
   * @param Byte
   * @return unsigned char
   */
  inline unsigned char returnLowBit(const int& Byte);

  /**
   * @brief 合并数据
   *
   * @param highbit   高八位数据
   * @param lowbit    低八位数据
   * @return int16_t  合并后数据
   */
  inline int16_t mergeIntoBytes(const unsigned char& highbit,
                                const unsigned char& lowbit);

  void writeData(const int& row, const int& col);
  bool Isopened();
  /**
   * @brief 发送串口
   * @param _keypoint 拐点集合
  */
  void writeData(std::vector<std::pair<uint8_t, uint8_t>> _keypoint);

  int receiveData1();
  
  int receiveData2();
  
  int ReturnReceive();

 private:
  Serial_Config serial_config_;
  int fd;
  int transform_arr_[4];
  unsigned char write_buff_[WRITE_BUFF_LENGTH];
  unsigned char buff_abcd[WRITE_BUFF_LENGTH];

  unsigned char receive_buff_[REC_INFO_LENGTH];
  unsigned char receive_buff_temp_[REC_INFO_LENGTH * 2];
  unsigned char exchangebyte_;

  int_union writerow_;
  int_union writecol_;
  int_union receivevision_;
  int16_t exchangebit_;

  ssize_t read_message_;
  ssize_t write_message_;

  /**
   * @brief Get the Data For Send object
   * @param  data_type        是否发现目标
   * @param  is_shooting      开火命令
   * @param  _yaw             yaw 符号
   * @param  yaw              yaw 绝对值
   * @param  _pitch           pitch 符号
   * @param  pitch            pitch 绝对值
   * @param  depth            深度
   * @param  CRC              CRC 校验码
   */
  void getDataForSend(const int& row, const int& col);
};

}  // namespace uart