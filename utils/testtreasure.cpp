#include "../src/treasure/include/treasure.h"
int main() {
  treasure::Treasure tr = treasure::Treasure("../config/treasure.xml");

  // cv::VideoCapture _cap(
  //     "/home/ling/Photoelectric_Competition_Scheme_2/treasure1.mp4");

  cv::VideoCapture _cap;
  for (int i = 3; i > -1; i--) {
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
  // _cap.set(cv::CAP_PROP_EXPOSURE, -13);

  int ret = tr.IdentifyTreasure(_cap);
  // cv::Mat frame;
  // std::vector<cv::Mat> channels;
  // cv::Mat B, R, G, B_R, B_G, R_B, R_G, G_R, G_B;
  // cv::Mat thre_b_r, thre_r_b;
  // while (true) {
  //   _cap >> frame;
  //   if (frame.empty() || cv::waitKey(50) == 27) {
  //     break;
  //   }
  //   cv::split(frame, channels);
  //   R = channels.at(2);
  //   B = channels.at(0);
  //   G = channels.at(1);
  //   B_R = B - R;
  //   R_B = R - B;
  //   G_R = G - R;
  //   cv::imshow("B-R", B_R);
  //   cv::imshow("G-R", G_R);
  //   R_B = R_B - 20;
  //   cv::imshow("R-B", R_B);
  //   cv::threshold(B_R, thre_b_r, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  //   cv::threshold(R_B, thre_r_b, 50, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  //   cv::morphologyEx(
  //       thre_r_b, thre_r_b, cv::MORPH_CLOSE,
  //       cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)),
  //       cv::Point(-1, -1), 3);
  //   cv::morphologyEx(
  //       thre_b_r, thre_b_r, cv::MORPH_CLOSE,
  //       cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)),
  //       cv::Point(-1, -1), 3);

  //   //  cv::adaptiveThreshold(R_B, thre_r_b, 255,
  //   //  cv::ADAPTIVE_THRESH_GAUSSIAN_C,
  //   //                     cv::THRESH_BINARY_INV, 31, 15);
  //   cv::imshow("thre_b-r", thre_b_r);
  //   cv::imshow("thre_r-b", thre_r_b);
  // }

  std::cout << ret << std::endl;
  return 0;
}