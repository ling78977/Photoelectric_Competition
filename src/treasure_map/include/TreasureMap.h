
#include <opencv2/opencv.hpp>

namespace treasuremap {

struct mapconfig {
  // IsQrPoint
  // 角点大小范围
  double point_minarea;
  double point_maxarea;
  // 定位点最小长宽比
  double minrate;

  // search_treasure
  // 宝藏点面积大小范围
  double treasure_minarea;
  double treasure_maxarea;
  // 宝藏点外接矩形最小长宽比
  double treasure_minrate;
  // 近似轮廓精度
  double precision;
  // 拟合圆形定点阈值
  int num_vertices;
  // 宝藏数量
  int num_treasures;

  // red_and_blue
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

  // 检查红蓝色块闭操作kernel
  int morphologray_kernel_size_close_1;
  int morphologray_kernel_size_close_2;

  // 检查红蓝色块开操作kernel
  int morphologray_kernel_size_open_1;
  int morphologray_kernel_size_open_2;

  int open_iterations;

  // 检测红蓝色块 最小面积
  int rb_minarea;
  // 红蓝色块最小长宽高比例
  double rb_minrate;

  // isCorner
  double ic_rate;

  int hsv_edit;
};

struct poses {
  // 宝藏图下方四个之一，从左到右数起第一个点,第一个int代表row，第二个int代表col，从零开始数
  std::pair<int, int> first_point;
  // 宝藏图下方四个之一，从左到右数起第二个点，第一个int代表row，第二个int代表col，从零开始数
  std::pair<int, int> second_point;
  // 宝藏图下方四个之一，从左到右数起第三个点，第一个int代表row，第二个int代表col，从零开始数
  std::pair<int, int> third_point;
  // 宝藏图下方四个之一，从左到右数起第四个点，第一个int代表row，第二个int代表col，从零开始数
  std::pair<int, int> fourth_point;
  // 宝藏图上方四个之一，从左到右数起第一个点，第一个int代表row，第二个int代表col，从零开始数
  std::pair<int, int> fivth_point;
  // 宝藏图上方四个之一，从左到右数起第二个点，第一个int代表row，第二个int代表col，从零开始数
  std::pair<int, int> sixth_point;
  // 宝藏图上方四个之一，从左到右数起第三个点，第一个int代表row，第二个int代表col，从零开始数
  std::pair<int, int> seventh_point;
  // 宝藏图上方四个之一，从左到右数起第四个点，第一个int代表row，第二个int代表col，从零开始数
  std::pair<int, int> eighth_point;
};

class TreatureMap {
 public:
  poses Returnpoes();
  bool Istreatures();
  TreatureMap(const std::string _treasuremap);
  /**
   * @brief 判断黑色与白色的比例
   * @param count 图像帧
   * @return 黑白比例
   */
  double Rate(cv::Mat &count);

  /**
   * @brief 处理二维码定位点
   * @param src 图像帧
   * @param rect 旋转矩形
   * @return 截取藏宝图的定位点roi
   */
  cv::Mat transformCorner(cv::Mat src, cv::RotatedRect rect);

  /**
   * @brief 判断定位点是否属于角上的正方形
   * @param image 截取图像帧
   * @return 是否属于角上的正方形
   */
  bool isCorner(cv::Mat &image);

  /**
   * @brief 检测是否是要找的定位点
   * @param contour 轮廓信息
   * @param img 截取图像帧率
   * @return 是否是要找的定位点
   */
  bool IsQrPoint(std::vector<cv::Point> &contour, cv::Mat &img);

  /**
   * @brief 识别宝藏点
   * @param frame 图像帧
   * @return 返回ros导航点类型的目标点集合
   */
  void search_treasure(cv::Mat &frame);

  /**
   * @brief 检测红蓝色块，并将图像旋转摆正
   * @param frame 截取图像帧
   */
  void red_and_blue(cv::Mat &frame);

  /**
   * @brief 完成宝藏图识别全过程
   * @param frame 传入图像帧
   */
  void process(cv::Mat &frame);

  void process1(cv::Mat &frame);
  void matrixmap();

  /**
   * @brief 释放内存
   */
  void release();
  ~TreatureMap();

 private:
  bool istreasures_ = false;
  mapconfig mapconfig_;
  poses poses_;

  std::vector<std::vector<cv::Point>> st_contours;
  std::vector<cv::Vec4i> st_hierarchy;
  std::vector<cv::Point2f> st_centers;

  std::vector<std::vector<cv::Point>> contours_red;
  std::vector<cv::Vec4i> hierarchy_red;
  std::vector<std::vector<cv::Point>> contours_blue;
  std::vector<cv::Vec4i> hierarchy_blue;

  // isCorner
  std::vector<std::vector<cv::Point>> ic_contours;
  std::vector<cv::Vec4i> ic_hierarchy;

  // process

  // std::vector<cv::Point> pro_center_all;
  std::vector<std::vector<cv::Point>> pro_contours;
  std::vector<cv::Vec4i> pro_hierarchy;
  std::vector<cv::Point2f> dst_corners = {
      cv::Point2f(0, 0), cv::Point2f(0, 600), cv::Point2f(600, 0),
      cv::Point2f(600, 600)};

  // 滑动条窗口名
  std::string blue_hsv_window_ = "BLUE_HSV_TRACKBAR";
  std::string red_hsv_window_ = "RED_HSV_TRACKBAR";

  const cv::Mat hsv_trackbar_red = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat hsv_trackbar_blue_ = cv::Mat::zeros(1, 300, CV_8UC1);
};
}  // namespace treasuremap
