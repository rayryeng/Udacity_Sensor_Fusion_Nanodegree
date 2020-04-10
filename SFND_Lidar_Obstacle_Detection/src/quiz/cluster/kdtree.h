/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef __KDTREE_H__
#define __KDTREE_H__
#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node* left;
  Node* right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}

  ~Node() {
    delete left;
    delete right;
  }
};

struct KdTree {
  Node* root;

  KdTree() : root(NULL) {}

  ~KdTree() {
    delete root;
    root = NULL;
  }

  void insert(std::vector<float> point, int id) {
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the
    // root

    // Base case if there are no nodes
    if (root == nullptr) {
      root = new Node(point, id);
      return;
    }

    // Point to beginning of the tree
    Node* ptr = root;

    // Record depth as we traverse
    int depth = 0;

    // Not doing this recursively - for practice
    // Until we hit a leaf...
    while (ptr != nullptr) {
      // Figure out which coordinate we want to compare with
      const int index = depth % point.size();
      const float coord_src = ptr->point[index];
      const float coord_to_comp = point[index];

      // If the incoming coordinate is less than the representative coordinate
      // of the node, then traverse left
      if (coord_to_comp < coord_src) {
        // When we go to the left, if there is nothing here,
        // place a node here and get out
        if (ptr->left == nullptr) {
          ptr->left = new Node(point, id);
          break;
        }
        // If not, traverse to the left
        ptr = ptr->left;
      } else if (coord_to_comp > coord_src) {
        // If the incoming coordinate is greater than the representative
        // coordinate of the node, then traverse right
        if (ptr->right == nullptr) {
          // When we go tot he right, if there is nothing here,
          // place a node here and get out
          ptr->right = new Node(point, id);
          break;
        }

        // If not, traverse to the right
        ptr = ptr->right;
      }

      // Increment depth
      depth++;
    }
  }

  // Recursive helper function to help us traverse through the tree
  void searchRecursiveHelper(std::vector<int>& ids,
                             const std::vector<float>& target,
                             const float& distanceTol,
                             const Node* node,
                             const int depth) {
    if (node == nullptr) {
      return;  // Don't do anything if we're at a leaf node
    }

    // Obtain the coordinate we want
    const int index = depth % target.size();
    const float coord_node = node->point[index];
    const float coord_target = target[index];

    // Check to see if all coordinates are within the tolerance (bounding box)
    bool check = true;
    for (size_t i = 0; i < target.size(); i++) {
      check &= (std::abs(node->point[index] - target[index]) <= distanceTol);
    }
    if (check) {
      // Now calculate the actual physical distance if it's within bounding box
      double dist = 0.0;
      for (size_t i = 0; i < target.size(); i++) {
        dist += (node->point[i] - target[i]) * (node->point[i] - target[i]);
      }
      if (dist <= (distanceTol * distanceTol)) { ids.push_back(node->id); }
    }

    // Now check the boundaries within distanceTol for the coordinate to
    // search for and go left and/or right if we need to
    if ((coord_target - distanceTol) < coord_node) {
      searchRecursiveHelper(ids, target, distanceTol, node->left, depth + 1);
    }
    if ((coord_target + distanceTol) > coord_node) {
      searchRecursiveHelper(ids, target, distanceTol, node->right, depth + 1);
    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;

    searchRecursiveHelper(ids, target, distanceTol, root, 0);
    return ids;
  }
};

#endif