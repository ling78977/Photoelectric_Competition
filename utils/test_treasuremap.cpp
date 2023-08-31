#include "../src/treasure_map/include/TreasureMap.h"

int main() {
  int grid[21][21] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1},
      {0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0},
      {0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0},
      {0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0},
      {0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0},
      {0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
      {0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0},
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
  treasuremap::TreatureMap tm =
      treasuremap::TreatureMap("../config/treasuremap.xml");

  // 创建一个VideoCapture对象
  // cv::VideoCapture cap =
  //     cv::VideoCapture("../map.mp4");
  cv::VideoCapture cap;
  for (int i = 3; i >-1 ; i--) {
    cap = cv::VideoCapture(i);
    if (cap.isOpened()) {
      break;
    } else {
      cap.release();
    }
  }
  if (!cap.isOpened()) {
    std::cerr << "\033[31m Couldn't open camera.\033[0m" << std::endl;
    return -1;
  } else {
    std::cout << "\033[1m open camera suceess .\033[0m" << std::endl;
  }
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  // 判断摄像头是否打开成功
  treasuremap::poses poses1;
  // 创建一个Mat对象
  cv::Mat frame;
  // 创建一个窗口
  cv::namedWindow("Camera", 1);
  // 循环读取和显示图像帧
  while (true) {
    // 从摄像头中读取一帧
    cap >> frame;
    // 判断图像是否为空
    if (frame.empty()) {
      break;
    }
    tm.process1(frame);
    // 在窗口中显示图像
    cv::imshow("Camera", frame);
    // 等待用户按键，如果按下q键，则退出循环
    if (cv::waitKey(30) == 27) {
      break;
    }
  }
  // 释放摄像头对象
  cap.release();
  // 销毁窗口
  cv::destroyAllWindows();
  return 0;
}