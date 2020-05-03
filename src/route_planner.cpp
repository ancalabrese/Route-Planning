#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model)
{
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
  CalculateHValue(start_node);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
  return node->distance(*end_node);
}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding
// all unvisited neighbors to the open list. Tips:
// - Use the FindNeighbors() method of the current_node to populate
// current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the
// g_value.
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and
// set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
  current_node->FindNeighbors();
  for (RouteModel::Node *n : current_node->neighbors)
  {
    if (n->visited)
    {
      continue;
    }
    n->parent = current_node;
    n->h_value = CalculateHValue(n);
    n->g_value = current_node->g_value + 1;
    n->visited = true;
    RoutePlanner::open_list.emplace_back(n);
  }
}

bool RoutePlanner::IsEnd(RouteModel::Node *node) { return &node == &end_node; }

RouteModel::Node *RoutePlanner::NextNode()
{
  RouteModel::Node *next_node = *(open_list.begin());
  for (RouteModel::Node *n : open_list)
  {
    if ((next_node->g_value + next_node->h_value) > (n->h_value + n->g_value))
    {
      next_node = n;
    }
  }
  open_list.erase(std::remove_if(open_list.begin(), open_list.end(), [next_node](RouteModel::Node *n) {
    return n->distance(*next_node) == 0;
  }));
  return next_node;
}

// TODO 6: Complete the ConstructFinalPath method to return the final path found
// from your A* search. Tips:
// - This method should take the current (final) node as an argument and
// iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to
// the distance variable.
// - The returned vector should be in the correct order: the start node should
// be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;
  while (current_node->distance(*start_node) != 0)
  {
    path_found.push_back(*current_node);
    current_node = current_node->parent;
  }
  

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of
                                     // the map to get meters.
  return path_found;
}

// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node
// to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method
// to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits.
// This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch()
{
  RouteModel::Node *current_node = start_node;
  current_node->visited = true;
  open_list.push_back(current_node);
  while (!open_list.empty())
  {
    current_node = NextNode();
    if (current_node->distance(*end_node) == 0)
    {
      m_Model.path = ConstructFinalPath(current_node);
      break;
    }
    else
    {
      AddNeighbors(current_node);
    }
  }
}