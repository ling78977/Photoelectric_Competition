#include <opencv2/opencv.hpp>

#include "./src/Astar/include/myAstar.h"
#include "./src/treasure/include/treasure.h"
#include "./src/uart/include/uart.h"
const std::string file = "../config/data.xml";
int main() {
  astart::MyAstar myastar;
  uart::SerialPort serial_ = uart::SerialPort("../config/uart.xml");
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
  if (!cap.isOpened() || !serial_.Isopened()) {
    if ((!cap.isOpened()) && (serial_.Isopened())) {
      std::cerr << "\033[31m Couldn't open camera.\033[0m" << std::endl;
    } else if ((cap.isOpened()) && (!serial_.Isopened())) {
      std::cerr << "\033[31m Couldn't open serial!\033[0m" << std::endl;
    } else {
      std::cerr << "\033[31m Couldn't open serial and camera.\033[0m"
                << std::endl;
    }
    return -1;
  }
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  std::vector<astart::Point> gol;

  cv::FileStorage fs(file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "\033[31mfailed open files!\033[0m" << std::endl;
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
  //   printf("\033[34m第%d个点:%d行,%d列\033[0m\n", i + 1, gol[i].row,
  //          gol[i].col);
  // }
  astart::Point curentp(19, 0);
  int bomped_nums = 0;  // 撞到了几个宝藏
  std::vector<std::pair<uint8_t, uint8_t>> Bomp;
  std::vector<std::pair<uint8_t, uint8_t>> Not_Bomp;
  Not_Bomp.push_back(std::pair<uint8_t, uint8_t>(88, 77));
  Bomp.push_back(std::pair<uint8_t, uint8_t>(77, 88));

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
    if (path.size() == 0) {
      continue;
    }
    keypoint = myastar.GetkeyPoint(path);

    // 发送串口
    serial_.writeData(keypoint);
    std::cout << "正在前往第" << i + 1 << "个点" << std::endl;
    // 等待下位机发送数据
    int a = serial_.receiveData1();
    // 打开摄像头识别宝藏
    int ret = tr.IdentifyTreasure(cap);
    treasure::result res;
    switch (ret) {
      case -1:
        res.ismycolor = treasure::UNKNOW;
        res.bomp = false;
        break;
      case 0:
        res.ismycolor = treasure::NOT;
        res.bomp = false;
        break;
      case 1:
        res.ismycolor = treasure::YES;
        res.bomp = false;
        break;
      case 2:
        res.ismycolor = treasure::YES;
        res.bomp = true;
        break;
    }
    // 如果识别到是我方颜色
    if (res.ismycolor == treasure::YES) {
      // 排除点
      switch (i) {
        case 0:
          // 如果这是第一个点，第二个点不用去，第八个点不用去
          gol[1].row = 0;
          gol[1].col = 0;
          gol[7].row = 0;
          gol[7].col = 0;
          break;
        case 1:
          // 如果这是第二个点，第七个点不用去
          gol[6].row = 0;
          gol[6].col = 0;
          break;
        case 2:
          // 如果这是第三个点，第四个点不用去，第六个点不用去
          gol[3].row = 0;
          gol[3].col = 0;
          gol[5].row = 0;
          gol[5].col = 0;
          break;
        case 3:
          // 如果这是第四个点，第五个点不用去
          gol[4].row = 0;
          gol[4].col = 0;
          break;
      }
    } else if (res.ismycolor == treasure::NOT) {  // 如果这不是我方颜色
                                                  // 排除点
      switch (i) {
        case 0:
          // 如果这是第一个点，第七个点不用去，
          gol[6].row = 0;
          gol[6].col = 0;
          break;
        case 1:
          // 如果这是第二个点，第八个点不用去
          gol[8].row = 0;
          gol[8].col = 0;
          break;
        case 2:
          // 如果这是第三个点，第五个点不用去
          gol[4].row = 0;
          gol[4].col = 0;
          break;
        case 3:
          // 如果这是第四个点，第六个点不用去
          gol[5].row = 0;
          gol[5].col = 0;
          break;
      }
    }
    if (res.bomp) {
      // 撞
      serial_.writeData(Bomp);
      bomped_nums++;
      // 更新起点,
      curentp = astart::Point(int(keypoint[keypoint.size() - 1].first),
                              int(keypoint[keypoint.size() - 1].second));
      std::cout << "撞，等待小车调整姿态" << std::endl;
      serial_.receiveData2();
    } else {
      // 不撞
      serial_.writeData(Not_Bomp);
      curentp = astart::Point(int(keypoint[keypoint.size() - 2].first),
                              int(keypoint[keypoint.size() - 2].second));
      std::cout << "不撞，等待小车调整姿态" << std::endl;
      serial_.receiveData2();
    }
  }

  // 去中终点

  std::vector<std::pair<uint8_t, uint8_t>> keypoint_togol;
  std::vector<astart::Point> path_togol;

  path_togol = myastar.findPath(curentp, gol[8]);
  keypoint_togol = myastar.GetkeyPoint(path_togol);
  keypoint_togol.push_back(std::pair<uint8_t, uint8_t>(1, 20));
  // 发送串口
  serial_.writeData(keypoint_togol);
  // 释放摄像头对象
  cap.release();
  // 销毁窗口
  cv::destroyAllWindows();
  return 0;
}