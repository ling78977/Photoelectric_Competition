#include "../include/treasure.h"

namespace treasure {

Treasure::Treasure(const std::string _treasureconfig) {
  cv::FileStorage config(_treasureconfig, cv::FileStorage::READ);
  if (!config.isOpened()) {
    std::cerr << "\033[31mfailed open treasure files!\033[0m" << std::endl;
  }
  config["H_BLUE_MIN"] >> treasureconfig_.h_blue_min;
  config["H_BLUE_MAX"] >> treasureconfig_.h_blue_max;
  config["S_BLUE_MIN"] >> treasureconfig_.s_blue_min;
  config["S_BLUE_MAX"] >> treasureconfig_.s_blue_max;
  config["V_BLUE_MIN"] >> treasureconfig_.v_blue_min;
  config["V_BLUE_MAX"] >> treasureconfig_.v_blue_max;
  config["H_RED1_MIN"] >> treasureconfig_.h_red1_min;
  config["H_RED1_MAX"] >> treasureconfig_.h_red1_max;
  config["S_RED1_MIN"] >> treasureconfig_.s_red1_min;
  config["S_RED1_MAX"] >> treasureconfig_.s_red1_max;
  config["V_RED1_MIN"] >> treasureconfig_.v_red1_min;
  config["V_RED1_MAX"] >> treasureconfig_.v_red1_max;
  config["H_RED2_MIN"] >> treasureconfig_.h_red2_min;
  config["H_RED2_MAX"] >> treasureconfig_.h_red2_max;
  config["S_RED2_MIN"] >> treasureconfig_.s_red2_min;
  config["S_RED2_MAX"] >> treasureconfig_.s_red2_max;
  config["V_RED2_MIN"] >> treasureconfig_.v_red2_min;
  config["V_RED2_MAX"] >> treasureconfig_.v_red2_max;
  config["H_GREEN_MIN"] >> treasureconfig_.h_green_min;
  config["H_GREEN_MAX"] >> treasureconfig_.h_green_max;
  config["S_GREEN_MIN"] >> treasureconfig_.s_green_min;
  config["S_GREEN_MAX"] >> treasureconfig_.s_green_max;
  config["V_GREEN_MIN"] >> treasureconfig_.v_green_min;
  config["V_GREEN_MAX"] >> treasureconfig_.v_green_max;
  config["H_YELLOW_MIN"] >> treasureconfig_.h_yellow_min;
  config["H_YELLOW_MAX"] >> treasureconfig_.h_yellow_max;
  config["S_YELLOW_MIN"] >> treasureconfig_.s_yellow_min;
  config["S_YELLOW_MAX"] >> treasureconfig_.s_yellow_max;
  config["L_YELLOW_MIN"] >> treasureconfig_.l_yellow_min;
  config["L_YELLOW_MAX"] >> treasureconfig_.l_yellow_max;
  config["MY_COLOR"] >> treasureconfig_.mycolor;
  config["TIME_OUT"] >> time_out_;
  config["SAMPLE_TIMES"] >> sample_times_;
  config["MORPHOLOGRAY_KERNEL_SIZE_CLOSE_1"] >>
      treasureconfig_.morphologray_kernel_size_close_1;
  config["MORPHOLOGRAY_KERNEL_SIZE_CLOSE_2"] >>
      treasureconfig_.morphologray_kernel_size_close_2;
  config["TREASURE_AREA_MIN"] >> treasureconfig_.treasure_area_min;
  config["TREASURE_RATE_MIN"] >> treasureconfig_.treasure_rate_min;
  config["TREASURE_RATE_MAX"] >> treasureconfig_.treasure_rate_max;
  config["JUDGEBOMP_RED"] >> treasureconfig_.judgebomp_red;
  config["MORPHOLOGRAY_KERNEL_SIZE_OPEN_1"] >>
      treasureconfig_.morphologray_kernel_size_open_1;
  config["MORPHOLOGRAY_KERNEL_SIZE_OPEN_2"] >>
      treasureconfig_.morphologray_kernel_size_open_2;
  config["KERNEL_SIZE_ERODE_1"] >> treasureconfig_.kernel_size_erode_1;
  config["KERNEL_SIZE_ERODE_2"] >> treasureconfig_.kernel_size_erode_2;
  config["KERNEL_SIZE_DILATE_1"] >> treasureconfig_.kernel_size_dilate_1;
  config["KERNEL_SIZE_DILATE_2"] >> treasureconfig_.kernel_size_dilate_2;
  config["ERODE_ITERATIONS"] >> treasureconfig_.erode_iterations;
  config["DILATE_ITERATIONS"] >> treasureconfig_.dilate_iterations;
  config["HSV_EDIT"] >> treasureconfig_.hsv_edit;
  config["CLOSE_ITERATIONS"] >> treasureconfig_.close_iterations;
  config["VALUE_GRAY"] >> treasureconfig_.value_gray;
  config["EPSILON"] >> treasureconfig_.epsilon;
  config["APPROX"] >> treasureconfig_.approx;
  config["CIRCLE_RATE"] >> treasureconfig_.circle_rate;
  config.release();

  if (treasureconfig_.hsv_edit == 1) {
    sample_times_ = 500000000000;
    time_out_ = 99999999;
    cv::namedWindow("EDIT", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("close_size_1", "EDIT",
                       &treasureconfig_.morphologray_kernel_size_close_1, 15,
                       NULL);
    cv::createTrackbar("close_size_2", "EDIT",
                       &treasureconfig_.morphologray_kernel_size_close_2, 15,
                       NULL);
    cv::createTrackbar("close_itrations", "EDIT",
                       &treasureconfig_.close_iterations, 10, NULL);
    cv::createTrackbar("value_gray", "EDIT", &treasureconfig_.value_gray, 150,
                       NULL);
  }
}
Treasure::~Treasure() {}
int Treasure::mode(std::vector<int>& nums) {
  std::unordered_map<int, int> count;  // 存储每个元素及其出现次数
  int max_count = 0;                   // 记录最大的次数
  int max_num = 0;                     // 记录最大次数对应的元素
  for (int num : nums) {               // 遍历容器
    count[num]++;                      // 更新哈希表
    if (count[num] > max_count) {  // 如果当前元素出现次数超过最大次数
      max_count = count[num];  // 更新最大次数
      max_num = num;           // 更新最大次数对应的元素
    } else if (
        count[num] == max_count &&
        num >
            max_num) {  // 如果当前元素出现次数等于最大次数，并且值大于之前的元素
      max_num = num;    // 更新最大次数对应的元素
    }
  }
  return max_num;  // 返回结果
}
int Treasure::IdentifyTreasure(cv::VideoCapture _cap) {
  double t0 = cv::getTickCount();
  cv::Mat img;
  std::vector<int> flags;
  int sample_time = 0;
  std::vector<cv::Mat> channels;
  cv::Mat B, R, B_R, R_B, R_B_2, thred_B_R, thred_G_R, thred_R_B, G, G_R;

  while (sample_time < sample_times_) {
    result result_;
    double task_time = (cv::getTickCount() - t0) / cv::getTickFrequency();
    if (task_time > double(time_out_) || cv::waitKey(100) == 27) {
      if (task_time > double(time_out_)) {
        std::cout << "\033[31m————————————超时,退出！————————————————\033[0m"
                  << std::endl;
      }
      break;
    }
    _cap >> img;
    if (img.empty()) {
      continue;
    }
    cv::Mat img_clone = img.clone();
    // 通道分离

    cv::split(img, channels);
    R = channels.at(2);
    B = channels.at(0);
    G = channels.at(1);
    B_R = B - R;
    R_B = R - B;
    G_R = G - R;
    R_B_2 = R - B - treasureconfig_.value_gray;
    cv::threshold(R_B_2, thred_R_B, 0, 255,
                  cv::THRESH_BINARY | cv::THRESH_OTSU);
    // cv::threshold(B_R, thred_B_R, 0, 255, cv::THRESH_BINARY |
    // cv::THRESH_OTSU);

    // cv::morphologyEx(
    //     G_R, thred_G_R, cv::MORPH_TOPHAT,
    //     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)),
    //     cv::Point(-1, -1), treasureconfig_.close_iterations);
    // cv::threshold(G_R, thred_G_R, 0, 255,
    //               cv::THRESH_BINARY_INV | cv::THRESH_TRIANGLE);

    cv::Mat thred_R_B_close, thred_B_R_close, thred_G_R_close;
    cv::morphologyEx(
        thred_R_B, thred_R_B_close, cv::MORPH_CLOSE,
        cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(treasureconfig_.morphologray_kernel_size_close_1,
                     treasureconfig_.morphologray_kernel_size_close_2)),
        cv::Point(-1, -1), treasureconfig_.close_iterations);
    // cv::morphologyEx(
    //     thred_B_R, thred_B_R_close, cv::MORPH_CLOSE,
    //     cv::getStructuringElement(
    //         cv::MORPH_ELLIPSE,
    //         cv::Size(treasureconfig_.morphologray_kernel_size_close_1,
    //                  treasureconfig_.morphologray_kernel_size_close_2)),
    //     cv::Point(-1, -1), treasureconfig_.close_iterations);
    // cv::morphologyEx(
    //     thred_G_R, thred_G_R_close, cv::MORPH_CLOSE,
    //     cv::getStructuringElement(
    //         cv::MORPH_ELLIPSE,
    //         cv::Size(treasureconfig_.morphologray_kernel_size_close_1,
    //                  treasureconfig_.morphologray_kernel_size_close_2)),
    //     cv::Point(-1, -1), treasureconfig_.close_iterations);

    if (treasureconfig_.hsv_edit == 1) {
      // cv::imshow("b", B);
      // cv::imshow("R", R);
      // cv::imshow("g-r", G_R);
      cv::imshow("thred_r-b_close", thred_R_B_close);
      // cv::imshow("thred_b-r_close", thred_B_R_close);
      // cv::imshow("r-b", R_B_2);
      // cv::imshow("thred_g-r", thred_G_R);
      // cv::imshow("thred_g-r_close", thred_G_R_close);
    }

    if (treasureconfig_.mycolor == RED) {  // 如果是红色就找矩形
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(thred_R_B_close, contours, hierarchy, cv::RETR_TREE,
                       cv::CHAIN_APPROX_SIMPLE);
      if (contours.size() != 0) {
        double max_area = 0;
        int max_index = -1;
        for (int i = 0; i < contours.size(); i++) {
          double area = cv::contourArea(contours[i]);
          cv::RotatedRect rect = cv::minAreaRect(contours[i]);
          double w = rect.size.width;
          double h = rect.size.height;
          // 设置近似精度为轮廓周长的0.01%
          double epsilon = 0.1 * arcLength(contours[i], true);
          // 进行多边形拟合
          std::vector<cv::Point> approx;
          cv::approxPolyDP(contours[i], approx, epsilon, true);
          double rate = cv::min(w, h) / cv::max(w, h);
          // std::cout << "approx:" << approx.size() << std::endl;
          if (area > treasureconfig_.treasure_area_min &&
              rate > treasureconfig_.treasure_rate_min &&
              rate < treasureconfig_.treasure_rate_max && area > max_area &&
              approx.size() == 4 &&
              area < img_clone.rows * img_clone.rows / 10) {
            max_area = area;
            max_index = i;
          }
        }
        if (max_index == -1) {  // 没有满足条件的轮廓，说明没有红色宝藏
          result_.bomp = false;
          result_.ismycolor = NOT;
        } else {  // 找到轮廓，判断是否有子轮廓，如果有，说明是红方真宝藏，否则是红方假宝藏
          cv::drawContours(img_clone, contours, max_index,
                           cv::Scalar(0, 255, 255), 2);
          if (hierarchy[max_index][2] != -1) {  // 有子轮廓

            result_.bomp = true;
            result_.ismycolor = YES;
          } else {  // 没有子轮廓
            result_.bomp = false;
            result_.ismycolor = YES;
          }
        }
      } else {  // 没有轮廓，就认为是蓝方宝藏
        result_.bomp = false;
        result_.ismycolor = NOT;
      }
      cv::imshow("img_clone", img_clone);
    } else if (treasureconfig_.mycolor == BLUE) {  // 找圆形
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(thred_R_B_close, contours, hierarchy, cv::RETR_EXTERNAL,
                       cv::CHAIN_APPROX_SIMPLE);
      // 先判断是否有红色宝藏
      if (contours.size() != 0) {
        double red_max_area = 0;
        int red_max_index = -1;
        for (int i = 0; i < contours.size(); i++) {
          double area = cv::contourArea(contours[i]);
          cv::RotatedRect rect = cv::minAreaRect(contours[i]);
          double w = rect.size.width;
          double h = rect.size.height;
          // 设置近似精度为轮廓周长的0.01%
          double epsilon = 0.1 * arcLength(contours[i], true);
          // 进行多边形拟合
          std::vector<cv::Point> approx;
          cv::approxPolyDP(contours[i], approx, epsilon, true);
          double rate = cv::min(w, h) / cv::max(w, h);
          // std::cout << "approx:" << approx.size() << std::endl;
          if (area > treasureconfig_.treasure_area_min &&
              rate > treasureconfig_.treasure_rate_min &&
              rate < treasureconfig_.treasure_rate_max && area > red_max_area &&
              approx.size() == 4 &&
              area < img_clone.rows * img_clone.rows / 10) {
            red_max_area = area;
            red_max_index = i;
          }
        }
        if (red_max_index != -1) {  // 有红色宝藏
          result_.bomp = false;
          result_.ismycolor = NOT;
        } else {  // 没有红色宝藏，再判断下蓝真还是蓝假
          double max_area = 0;
          int max_index = -1;
          for (int i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > 300 && area < img_clone.rows * img_clone.rows / 30) {
              cv::RotatedRect rect = cv::minAreaRect(contours[i]);
              double w = rect.size.width;
              double h = rect.size.height;
              double rate = cv::min(w, h) / cv::max(w, h);
              // 设置近似精度为轮廓周长的0.01%
              double epsilon =
                  treasureconfig_.epsilon * arcLength(contours[i], true);
              // 进行多边形拟合
              std::vector<cv::Point> approx;
              cv::approxPolyDP(contours[i], approx, epsilon, true);
              if (rate > treasureconfig_.circle_rate &&
                  approx.size() > treasureconfig_.approx) {
                std::cout << "area:" << area << std::endl;
                std::cout << "rate:" << rate << std::endl;
                std::cout << "approx.size():" << approx.size() << std::endl;
                max_area = area;
                max_index = i;
              }
            }
          }
          if (max_index == -1) {  // 没有该条件的轮廓,蓝色假
            result_.bomp = false;
            result_.ismycolor = YES;
          } else {  // 有
            cv::drawContours(img_clone, contours, max_index,
                             cv::Scalar(0, 255, 255), 2);
            result_.bomp = true;
            result_.ismycolor = YES;
          }
          cv::imshow("img_clone", img_clone);
        }
      } else {  // 没有轮廓就认为是蓝色假
        result_.bomp = false;
        result_.ismycolor = YES;
      }
    }
    sample_time++;
    if (result_.ismycolor == YES && result_.bomp == true) {
      flags.push_back(2);
    } else if (result_.ismycolor == YES && result_.bomp == false) {
      flags.push_back(1);
    } else if (result_.ismycolor == NOT && result_.bomp == false) {
      flags.push_back(0);
    } else if (result_.ismycolor == UNKNOW) {
      flags.push_back(-1);
    }
  }
  if (flags.size() == 0) {
    // 因为超时而什么都没有识别到
    return -1;
  } else {
    // 求最大众数
    return mode(flags);
  }
}

}  // namespace treasure
   // namespace treasure