#include "route_planner.h"

#include <algorithm>

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
  /*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  + Collect the neighbour "node" around the "current_node".            +
  + For each neighbor node fill the propertise h_value, g_value and    +
  + and set the current node as parent node of each neighbor node.     +
  + later the neghbor will move to the open list vector                +
  + ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
  current_node->RouteModel::Node::FindNeighbors();
  for (RouteModel::Node *node : current_node->neighbors) {
    if (node->visited) continue;
    node->h_value = CalculateHValue(node);
    node->g_value += current_node->distance(*node);
    node->visited = true;
    node->parent = current_node;
    RoutePlanner::open_list.emplace_back(node);
  }
}

RouteModel::Node *RoutePlanner::NextNode() {
  /*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  + To find RouteModel::Node belonging lowest f-value from RoutePlanner +
  + ::open_list list using bouble sort algotrithm.                      +
  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
  const int vec_size = RoutePlanner::open_list.size();
  RouteModel::Node *temp_node = nullptr;
  int Debug_i = 0;
  // bouble sort algorithm
  for (int i = 0; i < vec_size; i++) {
    for (int j = 0; j < (vec_size - 1); j++) {
      RouteModel::Node *preceding_node = RoutePlanner::open_list[j];
      double pre_f = preceding_node->g_value + preceding_node->h_value;

      RouteModel::Node *succeding_node = this->open_list[j + 1];
      double suc_f = succeding_node->g_value + succeding_node->h_value;

      if (pre_f < suc_f) {
        temp_node = preceding_node;
        RoutePlanner::open_list[j] = succeding_node;
        RoutePlanner::open_list[j + 1] = temp_node;
      }
    }
  }

  temp_node = RoutePlanner::open_list.back();  //[vec_size - 1]
  this->open_list.pop_back();

  return temp_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
    RouteModel::Node *current_node) {
  /*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  +  The final path will be constructed from this->end_node to  +
  +  to this->start_node every this->node reserves the          +
  +  RoutePlanner::Node::parent node and by tracking the parent +
  +  node the start_node could bereached. In each steps distance+
  +   between the crurrent node  and parent node will be        +
  +  calculated and will be added this-> distance which is the  +
  + total path distance                                         +
  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
  this->distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  while (current_node != this->start_node) {
    path_found.emplace_back(*current_node);
    RouteModel::Node *parent_n = current_node->parent;
    RoutePlanner::distance += current_node->distance(*parent_n);
    current_node = parent_n;
  }
  std::cout << "start node : " << start_node << "\n";
  std::cout << "Current node : " << current_node << "\n";
  path_found.emplace_back(*current_node);

  std::reverse(path_found.begin(), path_found.end());

  this->distance *= m_Model.MetricScale();  // Multiply the distance by the
                                            // scale of the map to get meters.
  return path_found;
}

void RoutePlanner::AStarSearch() {
  /*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  + Fill the attribute of RoundPlanner::start_node and add the   +
  + node in open_list. Find the lowest f_value node from open    +
  + list by NextNode() function. By the AddNeighbors() find the  +
  + neighbor nodes that were note visited already. Use the       +
  + process of finding lowest_f value nodes and their neighbors  +
  + until RoutePlanner::end_node is reached. Finally find the    +
  + full path using function RoutePlanner::ConstructFinalPath()  +
  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

  RouteModel::Node *start_n = RoutePlanner::start_node;
  start_n->visited = true;
  start_n->h_value = RoutePlanner::CalculateHValue(start_n);
  RoutePlanner::open_list.emplace_back(start_n);

  RouteModel::Node *current_node = nullptr;
  // std::vector<RouteModel::Node> final_path{};

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