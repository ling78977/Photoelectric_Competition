#include <opencv2/opencv.hpp>
namespace treasure {
/**
 * @param UNKNOW  -1 没有识别到宝藏
 * @param NOT  0 不是我方颜色的宝藏
 * @param YES  1 是我方颜色的宝藏
 */
enum Ismycolor {
  UNKNOW = -1,
  NOT = 0,
  YES = 1,
};

/**
 * @param RED 0 红色
 * @param BlUE 1 蓝色
 */
enum color {
  RED = 0,
  BLUE = 1,
};

/**
 * @brief 定义一个结果，表示识别宝藏得到的一些数据
 * @param ismycolr 是否是我方颜色 -1：不知道 0：不是 1：是
 * @param bomp 是否撞到宝藏
 */
struct result {
  int ismycolor = -1;  // 是否是我方的颜色
  bool bomp = false;   // 是否撞倒
};

/**
 * @brief 宝藏识别的一些参数
 */
struct treasureconfig {
  int h_blue_min;
  int h_blue_max;
  int s_blue_min;
  int s_blue_max;
  int v_blue_min;
  int v_blue_max;

  int h_red1_min;
  int h_red1_max;
  int s_red1_min;
  int s_red1_max;
  int v_red1_min;
  int v_red1_max;

  int h_red2_min;
  int h_red2_max;
  int s_red2_min;
  int s_red2_max;
  int v_red2_min;
  int v_red2_max;

  int h_yellow_min;
  int h_yellow_max;
  int l_yellow_min;
  int l_yellow_max;
  int s_yellow_min;
  int s_yellow_max;

  int h_green_min;
  int h_green_max;
  int s_green_min;
  int s_green_max;
  int v_green_min;
  int v_green_max;

  int mycolor;

  int morphologray_kernel_size_close_1;
  int morphologray_kernel_size_close_2;

  int morphologray_kernel_size_open_1;
  int morphologray_kernel_size_open_2;

  int kernel_size_erode_1;
  int kernel_size_erode_2;

  int kernel_size_dilate_1;
  int kernel_size_dilate_2;

  int erode_iterations;
  int dilate_iterations;
  int close_iterations;

  double treasure_area_min;
  double treasure_rate_min;
  double treasure_rate_max;

  double judgebomp_red;
  int hsv_edit;

  int value_gray;
  double epsilon;
  int approx;
  double circle_rate;
};

class Treasure {
 private:
  treasureconfig treasureconfig_;
  int time_out_;
  int sample_times_;

  // 滑动条窗口名
  const std::string blue_hsv_window_ = "BLUE_HSV_TRACKBAR";
  const std::string red_hsv_window_ = "RED_HSV_TRACKBAR";
  const std::string yellow_hsv_window_ = "YELLOW_HSV_TRACKBAR";
  const std::string green_hsv_window_ = "GREEN_HSV_TRACKBAR";
  const std::string treasurecof_ = "treasureconfig";

  const cv::Mat hsv_trackbar_red = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat hsv_trackbar_green_ = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat hsv_trackbar_blue_ = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat hsv_trackbar_yellow_ = cv::Mat::zeros(1, 300, CV_8UC1);
  // const cv::Mat treasure_area_min = cv::Mat::zeros(1, 300, CV_8UC1);
  // const cv::Mat treasure_rate_min = cv::Mat::zeros(1, 300, CV_8UC1);
  // const cv::Mat treasure_rate_min = cv::Mat::zeros(1, 300, CV_8UC1);

  /**
   * @brief 求一个容器里的众数
   * @param nums 装有数据的容器
   * @return 容器里的众数
   */
  int mode(std::vector<int>& nums);

 public:
  Treasure(const std::string _treasureconfig_);

  /**
   * @brief 判断是否要撞倒宝藏
   * @param _roi_hsv hsv图像帧的宝藏区域的roi图
   * @param _roi_src 原图像帧的宝藏区域的roi图
   * @return 判断结果
   */
  bool judgebomp(cv::Mat _roi_hsv, cv::Mat _roi_src);

  /**
   * @brief 识别宝藏
   * @param _cap 摄像头对象
   * @return
   * 返回一个flag用来表示识别宝藏的结果，
   * -1：不是到是不是我方颜色，不撞倒
   * 0：不是我方颜色的宝藏，不撞倒
   * 1：是我方颜色的宝藏，不撞倒
   * 2：是我方颜色的宝藏，撞倒
   */
  int IdentifyTreasure(cv::VideoCapture _cap);

  ~Treasure();
};

}  // namespace treasure