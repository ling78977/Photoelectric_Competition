#include <opencv2/opencv.hpp>

#include "../src/Astar/include/myAstar.h"
#include "../src/uart/include/uart.h"
const std::string file = "../config/data.xml";
int main() {
  astart::MyAstar myastar;
  uart::SerialPort serial_ = uart::SerialPort("../config/uart.xml");
  std::vector<std::vector<int>> grid = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1},
      {0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0},
      {0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0},
      {0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0},
      {0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0},
      {0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
      {0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0},
      {0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0},
      {0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
      {0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0},
      {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0},
      {0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0},
      {0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0},
      {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0},
      {0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0},
      {0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0},
      {0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0},
      {0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0},
      {1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  cv::Mat dst(21, 21, CV_8UC3, cv::Scalar(255, 255, 255));
  for (int i = 0; i < 21; i++) {
    for (int j = 0; j < 21; j++) {
      if (grid[i][j] == 0) {
        dst.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
      } else {
        dst.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }
  cv::Mat dst_ori;
  cv::resize(dst, dst_ori, cv::Size(0, 0), 40, 40, cv::INTER_NEAREST);
  cv::imshow("dst", dst_ori);

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
  static astart::Point curentp = astart::Point(19, 0);
  // cv::namedWindow("path");
  for (int i = 0; i < 8; i++) {
    if (gol[i].row == 0 && gol[i].col == 0) {
      continue;
    }
    std::vector<std::pair<uint8_t, uint8_t>> keypoint;
    std::vector<astart::Point> path;

    path = myastar.findPath(curentp, gol[i]);
    if (path.size() == 0) {
      continue;
    }
    std::cout << "第" << i + 1 << "个点" << std::endl;
    printf("当前起点：%d行 %d列\n", curentp.row, curentp.col);
    printf("当前终点：%d行 %d列\n", gol[i].row, gol[i].col);
    keypoint = myastar.GetkeyPoint(path);
    // 更新起点,求得的最后一个拐点（倒数第二个点）为下一次的起点
    curentp = astart::Point(int(keypoint[keypoint.size() - 1].first),
                            int(keypoint[keypoint.size() - 1].second));

    for (int j = 0; j < path.size(); j++) {
      cv::Mat dst_copy = dst.clone();
      dst_copy.at<cv::Vec3b>(path[j].row, path[j].col) = cv::Vec3b(0, 0, 255);
      // printf("第%d个点: %d行,%d列\n", i + 1, path[i].row, path[i].col);
      // cv::Mat path_tem;
      cv::resize(dst_copy, dst_copy, cv::Size(0, 0), 40, 40, cv::INTER_NEAREST);
      // dst_tem = path_tem;
      cv::imshow("path", dst_copy);
      cv::waitKey(0);
    }
  }

  return 0;
}