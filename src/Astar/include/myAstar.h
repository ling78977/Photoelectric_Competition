#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>

namespace astart {


/**
 * @brief 创建一个结构体用于拐点检测
 * @param up 是否可以向上走
 * @param down 是否可以向下走
 * @param left 是否可以像左走
 * @param right 是否可以向右走
*/
struct judekeypoint
{
  bool up;
  bool down;
  bool left;
  bool right;
};

/**
 * @brief 定义一个结构体来表示一个点坐标，定义了一个有参构造函数，和默认构造
 * @param row 第row行，从零开始数
 * @param col 第col列，从零开始数
 */
struct Point {
  // 行
  int row;
  // 列
  int col;
  Point(int _row, int _col) {
    row = _row;
    col = _col;
  }
  Point(){};
};

/**
 * @brief 定义一个结构体来表示栅格地图的节点,含有一个构造函数
 * @param point 节点坐标，表示第row行，第col列，从零开始数
 * @param g 从从起始节点到当前节点的代价
 * @param h  从当前节点到目标节点的启发式代价
 * @param f 总代价 (g + h)
 * @param parent 指向父节点的指针
 *
 */
struct Node {
  Point point;
  int g;
  int h;
  int f;
  Node* parent;
  Node(Point _point, int _g, int _h, Node* _parent)
      : point(_point), g(_g), h(_h), parent(_parent) {
    f = g + h;
  }
};

/**
 * @brief 定义一个比较函数来表示开放优先列表的优先级，f更小的优先级更高
 */
struct CompareF {
  bool operator()(const Node* a, const Node* b) const { return a->f > b->f; }
};

class MyAstar {
 private:
  // 一个向量来存储可能的移动 (上, 下, 左, 右)
  const std::vector<std::pair<int, int>> moves_ = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  // 一个二维向量来存储网格地图
  std::vector<std::vector<int>> grid_;
  // 一个二维向量来存储已访问过的节点
  std::vector<std::vector<bool>> visited_;
    // 一个优先队列来存储开放列表中的节点 (按 f 值排序)
    std::priority_queue<Node*, std::vector<Node*>, CompareF> open;

 public:
  MyAstar();
  /**
   * @brief 计算两个点之间的曼哈顿距离
   * @param _p1 点
   * @param _p2 点
   * @return 曼哈顿距离，两个点横向和纵向距离差的绝对值之和
   */
  int heuristic(Point _p1, Point _p2);

  /**
   * @brief 检查一个点是否有效 (在网格内且不是墙)
   * @param _p 一个节点的point属性
   * @return bool值
   */
  bool isValid(Point _p);

  /**
   * @brief 使用 A* 算法找到从起始节点到目标节点的最短路径
   * @param _startp 起点的point属性
   * @param _goalp 目标点的point属性
   * @return 得到的路径点集合
   */
  std::vector<astart::Point> findPath(Point _startp, Point _goalp);

  /**
   *@brief 通过传入路径点，获取拐点
   *@param _path 路径点的集合
   *@return 返回一个拐点的集合
   */
  std::vector<std::pair<uint8_t, uint8_t>> GetkeyPoint(
      std::vector<astart::Point> _path);

  ~MyAstar();
};

}  // namespace astart
