#ifndef KDTREE_H_
#define KDTREE_H_
#include <pcl/common/common.h>

#include <memory>

template <typename PointT>
class Node {
 public:
  Node(PointT point, int id);
  std::unique_ptr<Node> left;
  std::unique_ptr<Node> right;
  PointT point;
  int id;
};

template <typename PointT>
class KDTree {
 public:
  std::unique_ptr<Node<PointT>> root;
  KDTree();
  void insert(PointT point, int id);
  std::vector<int> search(PointT target, float distanceTol);

 private:
  void innerInsert(PointT point, int id, int depth, std::unique_ptr<Node<PointT>>& currentNode);
  void innerSearch(PointT target, float distanceTol, std::unique_ptr<Node<PointT>>& currentNode, std::vector<int>& indicies, int depth);
  bool pointInBox(PointT& target, PointT& point, float distanceTol);
  float pointIndex(PointT& point, int index);
};

#endif