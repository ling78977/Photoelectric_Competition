#include "../include/myAstar.h"

namespace astart {
std::vector<std::pair<uint8_t, uint8_t>> MyAstar::GetkeyPoint(
    std::vector<astart::Point> _path) {
  // 定义一个容器用于储存拐点
  // 因为数据较小，将int类型转换uint8_t类型
  std::vector<std::pair<uint8_t, uint8_t>> keypoint;
  // 遍历除去起点和终点的所有路径点

  for (int i = 1; i < _path.size() - 1; i++) {
    judekeypoint jude;
    // 遍历当前节点可以移动的方向
    for (int j = 0; j < 4; j++) {
      astart::Point next(_path[i].row + moves_[j].first,
                         _path[i].col + moves_[j].second);
      switch (j) {
        case 0:
          jude.up = grid_[next.row][next.col] > 0 ? true : false;
          // printf("up:%d\n",jude.up);
          break;
        case 1:
          jude.down = grid_[next.row][next.col] > 0 ? true : false;
          break;
        case 2:
          jude.left = grid_[next.row][next.col] > 0 ? true : false;
          break;
        case 3:
          jude.right = grid_[next.row][next.col] > 0 ? true : false;
          break;
      }
    }
    // 判断是否是路口（拐点）

    // 当不为横行通行也不为纵向通行时候，
    if ((!((jude.up) && (jude.down) && (!jude.left) && (!jude.right))) &&
        (!((!jude.up) && (!jude.down) && (jude.left) && (jude.right)))) {
      // 当前点为拐点
      printf("\033[32m第%d行,第%d列\033[0m\n", _path[i].row, _path[i].col);
      keypoint.push_back(std::pair<uint8_t, uint8_t>(uint8_t(_path[i].row),
                                                     uint8_t(_path[i].col)));
    }
  }
  keypoint.push_back(
      std::pair<uint8_t, uint8_t>(uint8_t(_path[_path.size() - 1].row),
                                  uint8_t(_path[_path.size() - 1].col)));

  return keypoint;
}
MyAstar::MyAstar() {
  grid_ = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
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
  // 初始化访问向量，用 false 值填充
  visited_.resize(grid_.size(), std::vector<bool>(grid_[0].size(), false));
}
int MyAstar::heuristic(Point _p1, Point _p2) {
  return abs(_p1.row - _p2.row) + abs(_p1.col - _p2.col);
}
// 一个方法来检查一个节点是否有效 (在网格内且不是墙)
bool MyAstar::isValid(Point _p) {
  return _p.col >= 0 && _p.col < grid_.size() && _p.row >= 0 &&
         _p.row < grid_[0].size() && grid_[_p.row][_p.col] == 1;
}
// 一个方法来使用 A* 算法找到从起始节点到目标节点的最短路径
std::vector<astart::Point> MyAstar::findPath(Point _startp, Point _goalp) {
  // 判断起点和终点是否是有效值
  if (isValid(_startp) && isValid(_goalp)) {
    // 重置visited_
    for (auto &v : visited_) {
      // 将每个 std::vector<bool> 都填充为 false
      std::fill(v.begin(), v.end(), false);
    }
    // 清空open
    while (!open.empty()) {
      open.pop();
    }

    // 创建一个起始节点和一个目标节点
    Node *start = new Node(_startp, 0, heuristic(_startp, _goalp), nullptr);
    Node *goal = new Node(_goalp, 0, 0, nullptr);

    // 将起始节点加入开放列表
    open.push(start);
    // 将起始节点标记为已访问
    visited_[start->point.row][start->point.col] = true;
    // 循环直到开放列表为空或者找到目标节点
    while (!open.empty()) {
      // 从开放列表中取出 f 值最小的节点
      Node *current = open.top();
      open.pop();
      // 检查当前节点是否是目标节点
      if (current->point.row == _goalp.row &&
          current->point.col == _goalp.col) {
        // 找到目标节点，通过回溯父节点构造路径
        std::vector<astart::Point> path;
        while (current != nullptr) {
          path.push_back(current->point);
          current = current->parent;
        }
        // 反转路径并返回
        std::reverse(path.begin(), path.end());
        //
        return path;
      }
      // 遍历当前节点可能的移动
      for (auto move : moves_) {
        // 通过加上移动偏移量得到下一个节点的坐标
        astart::Point next_point;
        next_point.row = current->point.row + move.first;
        next_point.col = current->point.col + move.second;
        // 检查下一个节点是否有效且未访问过
        if (isValid(next_point) && !visited_[next_point.row][next_point.col]) {
          // 创建一个新的节点，更新代价和父指针
          Node *next_node = new Node(next_point, current->g + 1,
                                     heuristic(next_point, _goalp), current);
          // 将下一个节点加入开放列表
          open.push(next_node);
          // 将下一个节点标记为已访问
          visited_[next_node->point.row][next_node->point.col] = true;
        }
      }
    }
    // 没有找到路径，返回一个空向量
    return {};
  } else {
    return {};
  }
}
MyAstar::~MyAstar() {}
}  // namespace astart
