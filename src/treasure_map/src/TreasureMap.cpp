#include "../include/TreasureMap.h"

namespace treasuremap {

poses TreatureMap::Returnpoes() { return poses_; }
bool TreatureMap::Istreatures() { return istreasures_; }
void TreatureMap::search_treasure(cv::Mat& frame) {
  cv::Mat st_gray;
  cv::Mat st_thred;
  cv::Mat src;
  src = frame.clone();
  cv::cvtColor(src, st_gray, cv::COLOR_BGR2GRAY);
  // equalizeHist(st_gray, st_gray);
  // cv::imshow("gray", st_gray);
  cv::blur(st_gray, st_gray, cv::Size(3, 3));
  cv::morphologyEx(st_gray, st_gray, cv::MORPH_BLACKHAT,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(21, 21)));
  // cv::imshow("blcak", st_gray);
  // 大津大法好
  cv::threshold(st_gray, st_thred, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  // cv::adaptiveThreshold(st_gray, st_thred, 255,
  // cv::ADAPTIVE_THRESH_GAUSSIAN_C,
  //                       cv::THRESH_BINARY_INV, 31, 15);
  // cv::imshow("thr", st_thred);
  cv::morphologyEx(st_thred, st_thred, cv::MORPH_OPEN,
                   cv::getStructuringElement(
                       cv::MORPH_ELLIPSE,
                       cv::Size(mapconfig_.morphologray_kernel_size_close_1,
                                mapconfig_.morphologray_kernel_size_close_1)),
                   cv::Point(-1, -1), 2);

  // 轮廓检测
  cv::findContours(st_thred, st_contours, st_hierarchy, cv::RETR_LIST,
                   cv::CHAIN_APPROX_NONE);
  // 对每个轮廓进行多边形拟合
  for (size_t i = 0; i < st_contours.size(); i++) {
    double area = cv::contourArea(st_contours[i]);
    if (mapconfig_.treasure_minarea < area &&
        area < mapconfig_.treasure_maxarea) {
      cv::RotatedRect rect = cv::minAreaRect(st_contours[i]);
      double w = rect.size.width;
      double h = rect.size.height;
      double rate = cv::min(w, h) / cv::max(w, h);
      if (rate > mapconfig_.treasure_minrate) {
        // 设置近似精度为轮廓周长的0.01%
        double epsilon = mapconfig_.precision * arcLength(st_contours[i], true);
        // 进行多边形拟合
        std::vector<cv::Point> approx;
        cv::approxPolyDP(st_contours[i], approx, epsilon, true);
        // cv::drawContours(frame, st_contours, i, cv::Scalar(0, 255, 0));
        // 如果拟合结果的顶点数大于6，表示是一个圆形
        if (approx.size() > mapconfig_.num_vertices) {
          // 在原图上绘制圆形
          cv::Point2f center;
          float radius;
          cv::minEnclosingCircle(st_contours[i], center, radius);
          st_centers.push_back(cv::Point2f(center.x / 500, center.y / 500));
          cv::circle(frame, center, radius, cv::Scalar(0, 0, 255), 2);
          cv::putText(frame, std::to_string(approx.size()), center, 1, 1,
                      cv::Scalar(0, 255, 0));
          cv::putText(frame, std::to_string(cvRound(area)),
                      cv::Point(center.x, center.y + 10), 1, 1,
                      cv::Scalar(255, 255, 0));
          cv::drawContours(frame, st_contours, i, cv::Scalar(0, 255, 0));
        }
      }
    }
  }
  cv::imshow("st_thred", st_thred);

  if (st_centers.size() == mapconfig_.num_treasures) {
    // 排序
    std::sort(st_centers.begin(), st_centers.end(),
              [](const cv::Point2f& p1, const cv::Point2f& p2) {
                return p1.y > p2.y;
              });
    std::sort(st_centers.begin(), st_centers.begin() + 4,
              [](const cv::Point2f& p1, const cv::Point2f& p2) {
                return p1.x < p2.x;
              });
    std::sort(st_centers.end() - 4, st_centers.end(),
              [](const cv::Point2f& p1, const cv::Point2f& p2) {
                return p1.x < p2.x;
              });
    // for (int i = 0; i < 4; i++) {
    //   st_centers.at(7 - i).x = 20 - st_centers.at(i).x;
    //   st_centers.at(7 - i).y = 20 - st_centers.at(i).y;
    // }
    std::cout << "找到v宝藏点:\n";
    cv::imshow("绘出宝藏点", frame);
    for (int i = 0; i < st_centers.size(); i++) {
      std::cout << st_centers[i].x << "," << st_centers[i].y << ";\n";
    }

    // 映射到矩阵21×21的矩阵
    matrixmap();
    istreasures_ = true;
  }
}

void TreatureMap::matrixmap() {
  // 以左上角为第零行第零列
  // 格式为第row行第col列
  for (int i = 0; i < mapconfig_.num_treasures; i++) {
    switch (i) {
      case 0:
        poses_.first_point.first = int(st_centers[i].y * 10) * 2 + 1;
        poses_.first_point.second = int(st_centers[i].x * 10) * 2 + 1;
        printf("\033[35m第1个点:  %d行 %d列\033[0m\n", poses_.first_point.first,
               poses_.first_point.second);
        break;

      case 1:
        poses_.second_point.first = int(st_centers[i].y * 10) * 2 + 1;
        poses_.second_point.second = int(st_centers[i].x * 10) * 2 + 1;
        printf("\033[35m第2个点:  %d行 %d列\033[0m\n",
               poses_.second_point.first, poses_.second_point.second);
        break;
      case 2:
        poses_.third_point.first = int(st_centers[i].y * 10) * 2 + 1;
        poses_.third_point.second = int(st_centers[i].x * 10) * 2 + 1;
        printf("\033[35m第3个点:  %d行 %d列\033[0m\n", poses_.third_point.first,
               poses_.third_point.second);
        break;
      case 3:
        poses_.fourth_point.first = int(st_centers[i].y * 10) * 2 + 1;
        poses_.fourth_point.second = int(st_centers[i].x * 10) * 2 + 1;
        printf("\033[35m第4个点:  %d行 %d列\033[0m\n",
               poses_.fourth_point.first, poses_.fourth_point.second);
        break;
      case 4:
        // poses_.fivth_point.first = int(st_centers[i].y * 10) * 2 + 1;
        // poses_.fivth_point.second = int(st_centers[i].x * 10) * 2 + 1;
        poses_.fivth_point.first = 20 - poses_.fourth_point.first;
        poses_.fivth_point.second = 20 - poses_.fourth_point.second;
        printf("\033[35m第5个点:  %d行 %d列\033[0m\n", poses_.fivth_point.first,
               poses_.fivth_point.second);
        break;
      case 5:
        // poses_.sixth_point.first = int(st_centers[i].y * 10) * 2 + 1;
        // poses_.sixth_point.second = int(st_centers[i].x * 10) * 2 + 1;
        poses_.sixth_point.first = 20 - poses_.third_point.first;
        poses_.sixth_point.second = 20 - poses_.third_point.second;
        printf("\033[35m第6个点:  %d行 %d列\033[0m\n", poses_.sixth_point.first,
               poses_.sixth_point.second);
        break;
      case 6:

        // poses_.seventh_point.first = int(st_centers[i].y * 10) * 2 + 1;
        // poses_.seventh_point.second = int(st_centers[i].x * 10) * 2 + 1;
        poses_.seventh_point.first = 20 - poses_.second_point.first;
        poses_.seventh_point.second = 20 - poses_.second_point.second;
        printf("\033[35m第7个点:  %d行 %d列\033[0m\n",
               poses_.seventh_point.first, poses_.seventh_point.second);
        break;
      case 7:
        // poses_.eighth_point.first = int(st_centers[i].y * 10) * 2 + 1;
        // poses_.eighth_point.second = int(st_centers[i].x * 10) * 2 + 1;
        poses_.eighth_point.first = 20 - poses_.first_point.first;
        poses_.eighth_point.second = 20 - poses_.first_point.second;
        printf("\033[35m第8个点:  %d行 %d列\033[0m\n",
               poses_.eighth_point.first, poses_.eighth_point.second);
        break;
    }
  }
}

// 检测红蓝色块
void TreatureMap::red_and_blue(cv::Mat& frame) {
  cv::Mat img_src = frame.clone();
  // cv::imshow("src",img_src);
  cv::Mat rb_hsv;
  cv::Mat rb_mask_red;
  cv::Mat rb_mask_blue;
  cv::Mat rb_mask_red1;
  cv::Mat rb_mask_red2;
  cv::Mat rb_blue_open;
  cv::Mat rb_red_close;
  cv::Mat rb_blue_and_red;

  cv::cvtColor(img_src, rb_hsv, cv::COLOR_BGR2HSV);
  // 蓝色
  cv::inRange(rb_hsv,
              cv::Scalar(mapconfig_.h_blue_min, mapconfig_.s_blue_min,
                         mapconfig_.v_blue_min),
              cv::Scalar(mapconfig_.h_blue_max, mapconfig_.s_blue_max,
                         mapconfig_.v_blue_max),
              rb_mask_blue);

  // 闭运算
  // std::cout << "mapconfig_.morphologray_kernel_size_close_1:"
  //           << mapconfig_.morphologray_kernel_size_close_1 << std::endl;
  // std::cout << "mapconfig_.minrate:" << mapconfig_.minrate << std::endl;

  cv::morphologyEx(rb_mask_blue, rb_blue_open, cv::MORPH_CLOSE,
                   cv::getStructuringElement(
                       cv::MORPH_RECT,
                       cv::Size(mapconfig_.morphologray_kernel_size_close_1,
                                mapconfig_.morphologray_kernel_size_close_1)));
  // cv::imshow("rb_open", rb_blue_open);
  // 红色
  cv::inRange(rb_hsv,
              cv::Scalar(mapconfig_.h_red1_min, mapconfig_.s_red1_min,
                         mapconfig_.v_red1_min),
              cv::Scalar(mapconfig_.h_red1_max, mapconfig_.s_red1_max,
                         mapconfig_.v_red1_max),
              rb_mask_red1);
  cv::inRange(rb_hsv,
              cv::Scalar(mapconfig_.h_red2_min, mapconfig_.s_red2_min,
                         mapconfig_.v_red2_min),
              cv::Scalar(mapconfig_.h_red2_max, mapconfig_.s_red2_max,
                         mapconfig_.v_red2_max),
              rb_mask_red2);
  cv::bitwise_or(rb_mask_red1, rb_mask_red2, rb_mask_red);

  cv::morphologyEx(
      rb_mask_blue, rb_blue_open, cv::MORPH_OPEN,
      cv::getStructuringElement(
          cv::MORPH_RECT, cv::Size(mapconfig_.morphologray_kernel_size_open_1,
                                   mapconfig_.morphologray_kernel_size_open_2)),
      cv::Point(-1, -1), mapconfig_.open_iterations);
  cv::imshow("blue_open1", rb_blue_open);
  // 闭运算
  cv::morphologyEx(rb_mask_red, rb_red_close, cv::MORPH_CLOSE,
                   cv::getStructuringElement(
                       cv::MORPH_RECT,
                       cv::Size(mapconfig_.morphologray_kernel_size_close_1,
                                mapconfig_.morphologray_kernel_size_close_2)));

  // 红蓝图合并
  cv::bitwise_or(rb_blue_open, rb_red_close, rb_blue_and_red);
  if (mapconfig_.hsv_edit) {
    cv::imshow("blue_open", rb_blue_open);
    cv::imshow("red_close", rb_red_close);
  }
  // 寻找两个色块
  bool flag_blue = false, flag_red = false, flag = false;
  cv::Rect blue_rect, red_rect;
  int rotate_angle;
  cv::Rect roirect;

  cv::findContours(rb_blue_open, contours_blue, hierarchy_blue,
                   cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  for (int i = 0; i < contours_blue.size(); i++) {
    double area = cv::contourArea(contours_blue[i]);
    // std::cout<<"蓝色面积："<<area<<"\n";
    if (area > mapconfig_.rb_minarea) {
      cv::RotatedRect rect = cv::minAreaRect(contours_blue[i]);
      double w = rect.size.width;
      double h = rect.size.height;
      double rate = cv::min(w, h) / cv::max(w, h);
      if (rate > mapconfig_.rb_minrate) {
        flag_blue = true;
        blue_rect = cv::boundingRect(cv::Mat(contours_blue[i]));
        break;
      }
    }
  }
  cv::findContours(rb_red_close, contours_red, hierarchy_red, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  for (int i = 0; i < contours_red.size(); i++) {
    double area = cv::contourArea(contours_red[i]);
    // std::cout<<"红色面积："<<area<<"\n";
    if (area > mapconfig_.rb_minarea) {
      cv::RotatedRect rect = cv::minAreaRect(contours_red[i]);
      double w = rect.size.width;
      double h = rect.size.height;
      double rate = std::min(w, h) / std::max(w, h);
      if (rate > mapconfig_.rb_minrate) {
        flag_red = true;
        red_rect = cv::boundingRect(cv::Mat(contours_red[i]));
        break;
      }
    }
  }
  if (flag_blue && flag_red) {
    flag = true;
  }
  if (!flag) {
    return;
  }
  // 判断红蓝色方块的相对位置
  // 若红色在上方
  if (red_rect.y < blue_rect.y) {
    // 红色在右上角
    if (red_rect.x > blue_rect.x) {
      rotate_angle = -360;  // cv::rotate()中表示旋转360度
      int width = red_rect.tl().x - blue_rect.br().x;
      int hight = blue_rect.br().y - red_rect.tl().y;
      roirect = cv::Rect(blue_rect.br().x, red_rect.tl().y, width, hight);
    } else {
      // 红色在左上角，顺时针旋转90度
      rotate_angle = -90;
      int width = blue_rect.br().x - red_rect.tl().x;
      int hight = blue_rect.tl().y - red_rect.br().y;
      roirect = cv::Rect(red_rect.tl().x, red_rect.tl().y + red_rect.height,
                         width, hight);
    }
    /*若蓝色在上方*/
  } else if (red_rect.y > blue_rect.y) {
    // 蓝色在右上角
    if (blue_rect.x > red_rect.x) {
      rotate_angle = -180;  // 可以不旋转
      int width = blue_rect.tl().x - red_rect.br().x;
      int hight = red_rect.br().y - blue_rect.tl().y;
      roirect = cv::Rect(red_rect.br().x, blue_rect.tl().y, width, hight);
    } else {
      // 蓝色在左上角
      rotate_angle = -270;
      int width = red_rect.br().x - blue_rect.tl().x;
      int hight = red_rect.tl().y - blue_rect.br().y;
      roirect = cv::Rect(blue_rect.tl().x, blue_rect.tl().y + blue_rect.height,
                         width, hight);
    }
  }
  // 截取图片，resize大小，旋转
  cv::Mat roiimg;
  roiimg = img_src(roirect);
  cv::imshow("roiimg", roiimg);
  cv::resize(roiimg, roiimg, cv::Size(500, 500));

  // 旋转
  cv::Mat M = cv::getRotationMatrix2D(cv::Point2f(250, 250), rotate_angle, 1);
  cv::warpAffine(roiimg, roiimg, M, cv::Size(500, 500));  // 应用仿射变换
  // cv::namedWindow("roi并旋转");
  cv::imshow("roi并旋转", roiimg);
  // cv::moveWindow("roi并旋转", 0, 480);
  search_treasure(roiimg);
}

// 释放内存
void TreatureMap::release() {
  // 释放search_treasure要用到的属性
  // poses.clear();
  // poses.shrink_to_fit();
  st_contours.clear();
  st_contours.shrink_to_fit();
  st_hierarchy.clear();
  st_hierarchy.shrink_to_fit();
  st_centers.clear();
  st_centers.shrink_to_fit();
  // 释放red_and_blue要用到的属性
  contours_red.clear();
  hierarchy_red.shrink_to_fit();
  contours_blue.clear();
  hierarchy_blue.shrink_to_fit();
  // process
  // pro_center_all.clear();
  // pro_center_all.shrink_to_fit();
  pro_contours.clear();
  pro_contours.shrink_to_fit();
  pro_hierarchy.clear();
  pro_hierarchy.shrink_to_fit();
}

TreatureMap::TreatureMap(const std::string _treasuremap) {
  cv::FileStorage _config(_treasuremap, cv::FileStorage::READ);
  if (!_config.isOpened()) {
    std::cerr << "\033[31mopen treasumap files failed !\033[0m" << std::endl;
  }
  _config["POINT_MINAREA"] >> mapconfig_.point_minarea;
  _config["POINT_MAXAREA"] >> mapconfig_.point_maxarea;
  _config["MINRATE"] >> mapconfig_.minrate;
  _config["TREASURE_MINAREA"] >> mapconfig_.treasure_minarea;
  _config["TREASURE_MAXAREA"] >> mapconfig_.treasure_maxarea;
  _config["TREASURE_MINRATE"] >> mapconfig_.treasure_minrate;
  _config["PRECISION"] >> mapconfig_.precision;
  _config["NUM_VERTICES"] >> mapconfig_.num_vertices;
  _config["NUM_TREASURES"] >> mapconfig_.num_treasures;
  _config["H_BLUE_MIN"] >> mapconfig_.h_blue_min;
  _config["H_BLUE_MAX"] >> mapconfig_.h_blue_max;
  _config["S_BLUE_MIN"] >> mapconfig_.s_blue_min;
  _config["S_BLUE_MAX"] >> mapconfig_.s_blue_max;
  _config["V_BLUE_MIN"] >> mapconfig_.v_blue_min;
  _config["V_BLUE_MAX"] >> mapconfig_.v_blue_max;
  _config["H_RED1_MIN"] >> mapconfig_.h_red1_min;
  _config["H_RED1_MAX"] >> mapconfig_.h_red1_max;
  _config["S_RED1_MIN"] >> mapconfig_.s_red1_min;
  _config["S_RED1_MAX"] >> mapconfig_.s_red1_max;
  _config["V_RED1_MIN"] >> mapconfig_.v_red1_min;
  _config["V_RED1_MAX"] >> mapconfig_.v_red1_max;
  _config["H_RED2_MIN"] >> mapconfig_.h_red2_min;
  _config["H_RED2_MAX"] >> mapconfig_.h_red2_max;
  _config["S_RED2_MIN"] >> mapconfig_.s_red2_min;
  _config["S_RED2_MAX"] >> mapconfig_.s_red2_max;
  _config["V_RED2_MIN"] >> mapconfig_.v_red2_min;
  _config["V_RED2_MAX"] >> mapconfig_.v_red2_max;
  _config["MORPHOLOGRAY_KERNEL_SIZE_CLOSE_1"] >>
      mapconfig_.morphologray_kernel_size_close_1;
  _config["MORPHOLOGRAY_KERNEL_SIZE_CLOSE_2"] >>
      mapconfig_.morphologray_kernel_size_close_2;
  _config["MORPHOLOGRAY_KERNEL_SIZE_OPEN_1"] >>
      mapconfig_.morphologray_kernel_size_open_1;
  _config["MORPHOLOGRAY_KERNEL_SIZE_OPEN_2"] >>
      mapconfig_.morphologray_kernel_size_open_2;
  _config["RB_MINAREA"] >> mapconfig_.rb_minarea;
  _config["RB_MINRATE"] >> mapconfig_.rb_minrate;
  _config["IC_RATE"] >> mapconfig_.ic_rate;
  _config["HSV_EDIT"] >> mapconfig_.hsv_edit;
  _config["OEPN_ITERATIONS"] >> mapconfig_.open_iterations;
  _config.release();
  if (mapconfig_.hsv_edit == 1) {
    // 创建滑动条
    cv::namedWindow(red_hsv_window_, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("h_red1_min", red_hsv_window_, &mapconfig_.h_red1_min,
                       180, NULL);
    cv::createTrackbar("h_red1_max", red_hsv_window_, &mapconfig_.h_red1_max,
                       180, NULL);
    cv::createTrackbar("s_red1_min", red_hsv_window_, &mapconfig_.s_red1_min,
                       255, NULL);
    cv::createTrackbar("s_red1_max", red_hsv_window_, &mapconfig_.s_red1_max,
                       255, NULL);
    cv::createTrackbar("v_red1_min", red_hsv_window_, &mapconfig_.v_red1_min,
                       255, NULL);
    cv::createTrackbar("v_red1_max", red_hsv_window_, &mapconfig_.v_red1_max,
                       255, NULL);

    cv::createTrackbar("h_red2_min", red_hsv_window_, &mapconfig_.h_red2_min,
                       180, NULL);
    cv::createTrackbar("h_red2_max", red_hsv_window_, &mapconfig_.h_red2_max,
                       180, NULL);
    cv::createTrackbar("s_red2_min", red_hsv_window_, &mapconfig_.s_red2_min,
                       255, NULL);
    cv::createTrackbar("s_red2_max", red_hsv_window_, &mapconfig_.s_red2_max,
                       255, NULL);
    cv::createTrackbar("v_red2_min", red_hsv_window_, &mapconfig_.v_red2_min,
                       255, NULL);
    cv::createTrackbar("v_red2_max", red_hsv_window_, &mapconfig_.v_red2_max,
                       255, NULL);

    cv::namedWindow(blue_hsv_window_, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("h_blue_min", blue_hsv_window_, &mapconfig_.h_blue_min,
                       180, NULL);
    cv::createTrackbar("h_blue_max", blue_hsv_window_, &mapconfig_.h_blue_max,
                       180, NULL);
    cv::createTrackbar("s_blue_min", blue_hsv_window_, &mapconfig_.s_blue_min,
                       255, NULL);
    cv::createTrackbar("s_blue_max", blue_hsv_window_, &mapconfig_.s_blue_max,
                       255, NULL);
    cv::createTrackbar("v_blue_min", blue_hsv_window_, &mapconfig_.v_blue_min,
                       255, NULL);
    cv::createTrackbar("v_blue_max", blue_hsv_window_, &mapconfig_.v_blue_max,
                       255, NULL);
  }
}
TreatureMap::~TreatureMap() {}

void TreatureMap::process1(cv::Mat& frame) {
  cv::namedWindow("canvas");
  cv::namedWindow("thred");
  cv::namedWindow("透视变换后");
  cv::Mat src;
  src = frame;
  // center_all获取特性中心个
  std::vector<cv::Point2f> pro_center_all;
  // cv::imshow("原图",src);
  cv::Mat srcCopy = src.clone();
  // canvas为画布 将找到的定位特征画出来
  cv::Mat canvas;
  canvas = cv::Mat::zeros(src.size(), CV_8UC3);
  cv::Mat srcGray;
  // 转化为灰度图
  cv::cvtColor(src, srcGray, cv::COLOR_BGR2GRAY);
  // cv::imshow("srcGray", srcGray);
  // 3X3模糊
  cv::blur(srcGray, srcGray, cv::Size(3, 3));
  // 计算直方图
  cv::convertScaleAbs(src, src);
  cv::adaptiveThreshold(srcGray, srcGray, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv::THRESH_BINARY_INV, 31, 15);
  // cv::morphologyEx(srcGray, srcGray, cv::MORPH_BLACKHAT,
  //                  cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15,
  //                  15)));
  // // imshow("黑帽", srcGray);
  // // 设置阈值根据实际情况 如视图中已找不到特征 可适量调整
  // cv::threshold(srcGray, srcGray, 0, 255, cv::THRESH_BINARY |
  // cv::THRESH_OTSU); cv::morphologyEx(srcGray, srcGray, cv::MORPH_CLOSE,
  //                  cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1)),
  //                  cv::Point(-1, -1), 1);
  // cv::imshow("thread", srcGray);
  //	用于轮廓检测
  cv::findContours(srcGray, pro_contours, pro_hierarchy, cv::RETR_TREE,
                   cv::CHAIN_APPROX_SIMPLE);
  int numOfRec = 0;
  for (int i = 0; i < pro_contours.size(); i++) {
    if (pro_hierarchy[i][2] != -1) {
      // 轮廓有子轮廓
      int child = pro_hierarchy[i][2];
      if (pro_hierarchy[child][2] != -1) {
        // 子轮廓有子轮廓
        int sonson = pro_hierarchy[child][2];
        if (pro_hierarchy[sonson][2] == -1) {
          // 子轮廓的子轮廓没有子轮廓

          // 判断下该轮廓的面积大小和长宽比率
          double area = cv::contourArea(pro_contours[i]);
          cv::RotatedRect rect = cv::minAreaRect(cv::Mat(pro_contours[i]));
          double w = rect.size.width;
          double h = rect.size.height;
          double rate = std::min(w, h) / std::max(w, h);
          if ((area < src.cols * src.rows / 100) && (rate > 0.5)) {
            cv::drawContours(src, pro_contours, i, cv::Scalar(0, 255, 0), 2, 8);
            cv::drawContours(canvas, pro_contours, i, cv::Scalar(0, 0, 255),
                             -1);
            pro_center_all.push_back(rect.center);
            numOfRec++;
          }
        }
      }
    }
  }
  cv::putText(canvas, std::to_string(numOfRec), cv::Point(10, 350),
              cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0, 0, 255), 5);
  cv::Mat dst(600, 600, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::putText(dst, "NO TARGET!", cv::Point(10, 350), cv::FONT_HERSHEY_SIMPLEX,
              3, cv::Scalar(0, 0, 255), 5);
  if (pro_center_all.size() == 4) {
    // 按照x大小排序
    std::sort(
        pro_center_all.begin(), pro_center_all.end(),
        [](const cv::Point& p1, const cv::Point& p2) { return p1.x < p2.x; });
    // 前两个和后两个点分别按照y排序
    std::sort(
        pro_center_all.begin(), pro_center_all.begin() + 2,
        [](const cv::Point& p1, const cv::Point& p2) { return p1.y < p2.y; });
    std::sort(
        pro_center_all.end() - 2, pro_center_all.end(),
        [](const cv::Point& p1, const cv::Point& p2) { return p1.y < p2.y; });
    // 连接四个正方形的部分
    for (int i = 0; i < pro_center_all.size(); i++) {
      cv::line(canvas, pro_center_all[i],
               pro_center_all[(i + 1) % pro_center_all.size()],
               cv::Scalar(255, 0, 0), 3);
    }
    try {
      // 透视变换
      cv::Mat dst1(600, 600, CV_8UC3, cv::Scalar(255, 255, 255));
      cv::Mat warpmatrx = cv::findHomography(pro_center_all, dst_corners);
      cv::warpPerspective(srcCopy, dst, warpmatrx, cv::Size(600, 600));
      // 将变换的图识别红蓝色块
      cv::imshow("透视变换后", dst);
      cv::moveWindow("透视变换后", srcGray.cols * 2, 0);
      // 识别红蓝色块，矫正方向，识别，宝藏点
      red_and_blue(dst);
    } catch (const std::exception& e) {
      std::cerr << e.what() << '\n';
    }
  } else {
    cv::imshow("透视变换后", dst);
    cv::moveWindow("透视变换后", srcGray.cols * 2, 0);
  }
  cv::imshow("thred", srcGray);
  cv::moveWindow("thred", 0, 0);
  cv::imshow("canvas", canvas);
  cv::moveWindow("canvas", canvas.cols, 0);
  release();
}

}  // namespace treasuremap
