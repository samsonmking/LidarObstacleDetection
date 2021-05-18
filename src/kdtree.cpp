#include "kdtree.h"

#include <memory>

template <typename PointT>
KDTree<PointT>::KDTree() {}

template <typename PointT>
void KDTree<PointT>::insert(PointT point, int id) {
  innerInsert(point, id, 0, root);
}

template <typename PointT>
void KDTree<PointT>::innerInsert(PointT point, int id, int depth, std::unique_ptr<Node<PointT>>& currentNode) {
  if (currentNode == nullptr) {
    currentNode.reset(new Node<PointT>(point, id));
    return;
  }

  int comp = depth % 3;
  float root_value = pointIndex(currentNode->point, comp);
  float point_value = pointIndex(point, comp);

  if (point_value < root_value) {
    innerInsert(point, id, ++depth, currentNode->left);
  } else {
    innerInsert(point, id, ++depth, currentNode->right);
  }
}

template <typename PointT>
std::vector<int> KDTree<PointT>::search(PointT target, float distanceTol) {
  std::vector<int> ids;
  innerSearch(target, distanceTol, root, ids, 0);
  return ids;
}

template <typename PointT>
void KDTree<PointT>::innerSearch(PointT target, float distanceTol, std::unique_ptr<Node<PointT>>& currentNode, std::vector<int>& ids, int depth) {
  if (currentNode == nullptr) {
    return;
  }
  if (pointInBox(target, currentNode->point, distanceTol)) {
    ids.push_back(currentNode->id);
  }
  int comp = depth % 3;
  if ((pointIndex(target, comp) - distanceTol) < pointIndex(currentNode->point, comp)) {
    innerSearch(target, distanceTol, currentNode->left, ids, depth + 1);
  }
  if ((pointIndex(target, comp) + distanceTol) > pointIndex(currentNode->point, comp)) {
    innerSearch(target, distanceTol, currentNode->right, ids, depth + 1);
  }
}

template <typename PointT>
bool KDTree<PointT>::pointInBox(PointT& target, PointT& point, float distanceTol) {
  float xDist = fabs(target.x - point.x);
  float yDist = fabs(target.y - point.y);
  float zDist = fabs(target.z - point.z);
  return xDist <= distanceTol && yDist <= distanceTol && zDist <= distanceTol && sqrt(pow(xDist, 2) + pow(yDist, 2) + pow(zDist, 2)) <= distanceTol;
}

template <typename PointT>
float KDTree<PointT>::pointIndex(PointT& point, int index) {
  switch (index) {
    case 0:
      return point.x;
    case 1:
      return point.y;
    default:
      return point.z;
  }
}

template <typename PointT>
Node<PointT>::Node(PointT point, int id) : point(point), id(id) {}