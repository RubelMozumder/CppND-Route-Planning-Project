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
  return node->distance((*end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (RouteModel::Node *node : current_node->neighbors) {
    node->h_value = CalculateHValue(node);
    node->g_value = node->g_value + 1;
    node->parent = current_node;
    node->visited = true;
    RoutePlanner::open_list.emplace_back(node);
  }
}

/*
TODO myself: Introduce a function sort
node_list() that will take a node list as reference and sort it by
condition of a function given as argument.
*/
RouteModel::Node *RoutePlanner::NextNode() {
  int vec_size = RoutePlanner::open_list.size();
  RouteModel::Node *temp_node = nullptr;

  // bouble sort algorithm
  for (int i = 0; i < vec_size; i++) {
    for (int j = 0; j < (vec_size - 1); j++) {
      RouteModel::Node *preceding_node = open_list[j];
      double pre_f = preceding_node->g_value + preceding_node->h_value;

      RouteModel::Node *succeding_node = open_list[j + 1];
      double suc_f = succeding_node->g_value + succeding_node->h_value;

      if (pre_f < suc_f) {
        temp_node = preceding_node;
        preceding_node = succeding_node;
        succeding_node = temp_node;
      }
    }
  }
  temp_node = open_list[vec_size - 1];
  open_list.pop_back();

  return temp_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
    RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  while (current_node != start_node) {
    path_found.emplace_back(*current_node);
    RouteModel::Node parent_n = *(current_node->parent);
    RoutePlanner::distance += current_node->distance(parent_n);
    path_found.emplace_back(parent_n);
    current_node = current_node->parent;
  }

  // TODO myself : Apply the sortation function that has been implemented for
  // NextNode().

  std::reverse(path_found.begin(), path_found.end());

  distance *= m_Model.MetricScale();  // Multiply the distance by the scale of
                                      // the map to get meters.
  return path_found;
}

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = RoutePlanner::start_node;
  std::vector<RouteModel::Node> final_path{};
  while (current_node != RoutePlanner::end_node) {
    AddNeighbors(current_node);
    current_node = NextNode();

    if (current_node == RoutePlanner::end_node) {
      final_path = ConstructFinalPath(current_node);
      break;
    }
  }
  RoutePlanner::m_Model.path = final_path;
}