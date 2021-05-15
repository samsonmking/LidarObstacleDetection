/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node* left;
  Node* right;

  Node(std::vector<float> arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {}

  ~Node() {
    delete left;
    delete right;
  }
};

struct KdTree {
  Node* root;

  KdTree() : root(NULL) {}

  ~KdTree() { delete root; }

  void insert(std::vector<float> point, int id, int depth = 0) { innerInsert(point, id, 0, root); }

  void innerInsert(std::vector<float> point, int id, int depth, Node*& currentNode) {
    if (currentNode == NULL) {
      currentNode = new Node(point, id);
      return;
    }

    int comp = depth % 2;
    float root_value = currentNode->point[comp];
    float point_value = point[comp];

    if (point_value < root_value) {
      innerInsert(point, id, ++depth, currentNode->left);
    } else {
      innerInsert(point, id, ++depth, currentNode->right);
    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    innerSearch(target, distanceTol, ids, root, 0);
    return ids;
  }

  void innerSearch(std::vector<float> target, float distanceTol, std::vector<int>& ids, Node*& currentNode, int depth) {
    if (currentNode == NULL) {
      return;
    }
    if (pointInBox(target, currentNode->point, distanceTol)) {
      ids.push_back(currentNode->id);
    }
    int comp = depth % 2;
    if ((target[comp] - distanceTol) < currentNode->point[comp]) {
      innerSearch(target, distanceTol, ids, currentNode->left, depth + 1);
    }
    if ((target[comp] + distanceTol) > currentNode->point[comp]) {
      innerSearch(target, distanceTol, ids, currentNode->right, depth + 1);
    }
  }

  bool pointInBox(std::vector<float>& target, std::vector<float>& point, float distanceTol) {
    float xDist = fabs(target[0] - point[0]);
    float yDist = fabs(target[1] - point[1]);
    return xDist <= distanceTol && yDist <= distanceTol && sqrt(pow(xDist, 2) + pow(yDist, 2)) < distanceTol;
  }
};
