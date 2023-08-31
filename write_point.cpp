#include "./src/Astar/include/myAstar.h"
#include "./src/treasure/include/treasure.h"
#include "./src/treasure_map/include/TreasureMap.h"
#include "./src/uart/include/uart.h"

int main() {
  astart::MyAstar myastar;
  uart::SerialPort serial_ = uart::SerialPort("../config/uart.xml");
  treasuremap::TreatureMap tm =
      treasuremap::TreatureMap("../config/treasuremap.xml");
  treasure::Treasure tr = treasure::Treasure("../config/treasure.xml");
  cv::VideoCapture _cap;
  for (int i = 0; i < 4; i++) {
    _cap = cv::VideoCapture(i);
    if (_cap.isOpened()) {
      break;
    } else {
      _cap.release();
    }
  }
  if (!_cap.isOpened()) {
    std::cerr << "\033[31m Couldn't open camera.\033[0m" << std::endl;
    return -1;
  } else {
    std::cout << "\033[1m open camera suceess .\033[0m" << std::endl;
  }
  _cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  _cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  cv::FileStorage fs("../config/data.xml", cv::FileStorage::WRITE, "xml");
  if (!fs.isOpened()) {
    std::cout << "\033[31m coudn't open data files! \033[0m" << std::endl;
    return -1;
  }
  treasuremap::poses gol_poes;
  // 创建一个Mat对象
  cv::Mat frame;
  // 创建一个窗口
  cv::namedWindow("Camera", 1);
  // 循环读取和显示图像帧
  while (true) {
    // 从摄像头中读取一帧
    _cap >> frame;
    // 判断图像是否为空
    if (frame.empty()) {
      break;
    }
    tm.process1(frame);
    if (tm.Istreatures()) {
      gol_poes = tm.Returnpoes();
      fs << "first_point_row" << gol_poes.first_point.first;
      fs << "first_point_col" << gol_poes.first_point.second;
      fs << "second_point_row" << gol_poes.second_point.first;
      fs << "second_point_col" << gol_poes.second_point.second;
      fs << "third_point_row" << gol_poes.third_point.first;
      fs << "third_point_col" << gol_poes.third_point.second;
      fs << "fourth_point_row" << gol_poes.fourth_point.first;
      fs << "fourth_point_col" << gol_poes.fourth_point.second;
      fs << "fivth_point_row" << gol_poes.fivth_point.first;
      fs << "fivth_point_col" << gol_poes.fivth_point.second;
      fs << "sixth_point_row" << gol_poes.sixth_point.first;
      fs << "sixth_point_col" << gol_poes.sixth_point.second;
      fs << "seventh_point_row" << gol_poes.seventh_point.first;
      fs << "seventh_point_col" << gol_poes.seventh_point.second;
      fs << "eighth_point_row" << gol_poes.eighth_point.first;
      fs << "eighth_point_col" << gol_poes.eighth_point.second;
      break;
    }
    // 在窗口中显示图像
    // cv::imshow("Camera", frame);
    // 等待用户按键，如果按下q键，则退出循环
    if (cv::waitKey(30) == 27) {
      break;
    }
  }
  // 释放摄像头对象
  _cap.release();
  fs.release();
  // 销毁窗口
  cv::destroyAllWindows();
  return 0;
}