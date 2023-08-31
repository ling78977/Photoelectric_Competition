#include <opencv2/opencv.hpp>
#include <typeinfo>

#include "./src/Astar/include/myAstar.h"
#include "./src/treasure/include/treasure.h"
#include "./src/uart/include/uart.h"
const std::string file = "./config/data.xml";
int main() {
  astart::MyAstar myastar;
  // uart::SerialPort serial_ = uart::SerialPort("../config/uart.xml");
  treasure::Treasure tr = treasure::Treasure("../config/treasure.xml");
  // 创建一个VideoCapture对象
  cv::VideoCapture cap;
  for (int i = 0; i < 4; i++) {
    cap = cv::VideoCapture(i);
    if (cap.isOpened()) {
      break;
    } else {
      cap.release();
    }
  }
  // 判断摄像头是否打开成功
  // if (!cap.isOpened() || !serial_.Isopened()) {
  //   if ((!cap.isOpened()) && (serial_.Isopened())) {
  //     std::cerr << "\033[31m Couldn't open camera.\033[0m" << std::endl;
  //   } else if ((cap.isOpened()) && (!serial_.Isopened())) {
  //     std::cerr << "\033[31m Couldn't open serial!\033[0m" << std::endl;
  //   } else {
  //     std::cerr << "\033[31m Couldn't open serial and camera.\033[0m"
  //               << std::endl;
  //   }
  //   // return -1;
  // }
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  std::vector<astart::Point> gol;

  cv::FileStorage fs(file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "\033[31m failed open data files!\033[0m" << std::endl;
    return -1;
  }
  gol.push_back(astart::Point(fs["first_point_row"], fs["first_point_col"]));
  gol.push_back(astart::Point(fs["second_point_row"], fs["second_point_col"]));
  gol.push_back(astart::Point(fs["third_point_row"], fs["third_point_col"]));
  gol.push_back(astart::Point(fs["fourth_point_row"], fs["fourth_point_col"]));
  gol.push_back(astart::Point(fs["fivth_point_row"], fs["fivth_point_col"]));
  gol.push_back(astart::Point(fs["sixth_point_row"], fs["sixth_point_col"]));
  gol.push_back(
      astart::Point(fs["seventh_point_row"], fs["seventh_point_col"]));
  gol.push_back(astart::Point(fs["eighth_point_row"], fs["eighth_point_col"]));
  // 终点
  gol.push_back(astart::Point(1, 20));
  fs.release();
  // for (int i = 0; i < 9; i++) {
  //   printf("第%d个点:%d行,%d列\n", i + 1, gol[i].row, gol[i].col);
  // }
  static astart::Point curentp(19, 0);
  int bomped_nums = 0;  // 撞到了几个宝藏
  for (int i = 0; i < 8; i++) {
    if (bomped_nums == 3) {
      break;
    }
    if (gol[i].row == 0 && gol[i].col == 0) {
      continue;
    }
    std::vector<std::pair<uint8_t, uint8_t>> keypoint;
    std::vector<astart::Point> path;

    path = myastar.findPath(curentp, gol[i]);
    keypoint = myastar.GetkeyPoint(path);
    // 更新起点,求得的最后一个拐点（倒数第二个点）为下一次的起点
    curentp = astart::Point(int(keypoint[keypoint.size() - 2].first),
                            int(keypoint[keypoint.size() - 2].second));
    // 发送串口
    // serial_.writeData(keypoint);
    printf("------------第%d次发送，等待接受数据------------\n", i + 1);
    // serial_.receiveData2();
  }

  // 去中终点

  std::vector<std::pair<uint8_t, uint8_t>> keypoint_togol;
  std::vector<astart::Point> path_togol;

  path_togol = myastar.findPath(curentp, gol[8]);
  keypoint_togol = myastar.GetkeyPoint(path_togol);
  keypoint_togol.push_back(std::pair<uint8_t, uint8_t>(1, 20));
  // 发送串口
  // serial_.writeData(keypoint_togol);
  // 释放摄像头对象
  cap.release();
  // 销毁窗口
  cv::destroyAllWindows();
  return 0;
}