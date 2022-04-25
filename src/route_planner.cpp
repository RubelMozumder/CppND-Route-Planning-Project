#include "route_planner.h"

#include <algorithm>
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+ Route Planner (A*Search) algorithm:                                         +
+ 1. To find nearest node around Starting and ending location.                +
+ 2. To search and collect neighboring nodes around the starting node(current +
+ node). 3- To set current node as parent node of the founded neighboring     +
+ nodes.                                                                      +
+ 4. Choose next current node from neighboring node depending on lowest (g+f) +
+ value.                                                                      +
+ 5. Continue steps 2-4 until we reach end node.                              +
+ 6. Finally path nodes from end node to start node by tracking parent node.  +
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  RoutePlanner::start_node = &m_Model.FindClosestNode(start_x, start_y);
  RoutePlanner::end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*(this->end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->RouteModel::Node::FindNeighbors();
  for (auto &node : current_node->neighbors) {
    if (node->visited) continue;
    node->h_value = CalculateHValue(node);
    node->g_value = current_node->g_value + current_node->distance(*node);
    node->visited = true;
    node->parent = current_node;
    RoutePlanner::open_list.emplace_back(node);
  }
}

RouteModel::Node *RoutePlanner::NextNode() {
  const int vec_size = RoutePlanner::open_list.size();
  RouteModel::Node *temp_node = nullptr;
  int Debug_i = 0;

  std::sort(RoutePlanner::open_list.begin(), RoutePlanner::open_list.end(),
            [](RouteModel::Node *a, RouteModel::Node *b) {
              return (a->g_value + a->h_value > b->g_value + b->h_value);
            });

  temp_node = RoutePlanner::open_list.back();  //[vec_size - 1]
  this->open_list.pop_back();

  return temp_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
    RouteModel::Node *current_node) {
  this->distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  while (current_node != this->start_node) {
    path_found.emplace_back(*current_node);
    RouteModel::Node *parent_n = current_node->parent;
    RoutePlanner::distance += current_node->distance(*parent_n);
    current_node = parent_n;
  }

  path_found.emplace_back(*current_node);

  std::reverse(path_found.begin(), path_found.end());

  this->distance *= m_Model.MetricScale();  // Multiply the distance by the
                                            // scale of the map to get meters.
  return path_found;
}

void RoutePlanner::AStarSearch() {
  RouteModel::Node *start_n = RoutePlanner::start_node;
  start_n->visited = true;
  start_n->h_value = RoutePlanner::CalculateHValue(start_n);
  RoutePlanner::open_list.emplace_back(start_n);

  RouteModel::Node *current_node = nullptr;

  while (!RoutePlanner::open_list.empty()) {
    current_node = RoutePlanner::NextNode();
    if (current_node == RoutePlanner::end_node) {
      RoutePlanner::m_Model.path =
          RoutePlanner::ConstructFinalPath(current_node);
      return;
    }
    AddNeighbors(current_node);
  }
}