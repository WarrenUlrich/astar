#pragma once

#include <algorithm>
#include <concepts>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <tuple>
#include <unordered_map>
#include <variant>
#include <vector>

namespace astar {

template <typename T>
concept node = requires(T t) {
  { std::hash<T>{}(t) } -> std::convertible_to<std::size_t>;
  { t == t } -> std::convertible_to<bool>;
};

template <typename T, typename Node>
concept cost_fn = requires(T t, Node n1, Node n2) {
  { t(n1, n2) } -> std::convertible_to<double>;
};

template <typename T, typename Node>
concept heuristic_fn = requires(T t, Node n1, Node n2) {
  { t(n1, n2) } -> std::convertible_to<double>;
};

template <typename T, typename Node>
concept neighbors_fn = requires(T t, Node n) {
  { t(n) } -> std::ranges::range;
  requires std::same_as<
      std::ranges::range_value_t<decltype(t(n))>, Node>;
};

template <node Node, cost_fn<Node> CostFn,
          heuristic_fn<Node> HeuristicFn,
          neighbors_fn<Node> NeighborsFn>
class pathfinder {
public:
  using path = std::vector<Node>;

  pathfinder(CostFn cost_fn, HeuristicFn heuristic_fn,
             NeighborsFn neighbors_fn)
      : _cost_fn(cost_fn), _heuristic_fn(heuristic_fn),
        _neighbors_fn(neighbors_fn) {}

  std::optional<path> search(Node start, Node goal) {
    if (start == goal) {
      return path{
          start}; // If start is goal, return it immediately
    }

    std::unordered_map<Node, Node> came_from;
    std::unordered_map<Node, double> cost_so_far;

    auto cmp = [&](const Node &a, const Node &b) {
      return _heuristic_fn(a, goal) <
             _heuristic_fn(b, goal);
    };

    std::set<Node, decltype(cmp)> frontier(cmp);

    auto push_or_update = [&](Node node, double priority) {
      auto [it, inserted] = frontier.insert(node);
      if (!inserted) {
        if (priority < cost_so_far[node]) {
          frontier.erase(it);
          frontier.insert(node);
        }
      }
    };

    push_or_update(start, 0);
    came_from[start] = start;
    cost_so_far[start] = 0.0;

    while (!frontier.empty()) {
      Node current = *frontier.begin();
      frontier.erase(frontier.begin());

      if (current == goal) {
        path total_path;
        while (current != start) {
          total_path.push_back(current);
          current = came_from[current];
        }
        total_path.push_back(start);
        std::reverse(total_path.begin(), total_path.end());
        return total_path;
      }

      for (const Node &next : _neighbors_fn(current)) {
        double new_cost =
            cost_so_far[current] + _cost_fn(current, next);
        if (!cost_so_far.count(next) ||
            new_cost < cost_so_far[next]) {
          cost_so_far[next] = new_cost;
          push_or_update(
              next, new_cost + _heuristic_fn(next, goal));
          came_from[next] = current;
        }
      }
    }

    return std::nullopt; // If there's no path to the goal
  }

private:
  CostFn _cost_fn;
  HeuristicFn _heuristic_fn;
  NeighborsFn _neighbors_fn;
};

} // namespace astar
