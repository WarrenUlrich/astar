
#include <astar/astar.hpp>
#include <iostream>

class tile {
public:
  int x;
  int y;

  constexpr tile() : x(-1), y(-1) {}

  tile(int x, int y) : x(x), y(y) {}

  bool operator==(const tile &other) const {
    return x == other.x && y == other.y;
  }

  int distance(const tile &other) const {
    return std::abs(x - other.x) + std::abs(y - other.y);
  }
};

namespace std {
template <> struct hash<tile> {
  std::size_t operator()(const tile &t) const {
    return std::hash<int>{}(t.x) ^ std::hash<int>{}(t.y);
  }
};
} // namespace std

class tile_cost_fn {
public:
  double operator()(const tile &t1, const tile &t2) const {
    return t1.distance(t2);
  }
};

class tile_heuristic_fn {
public:
  double operator()(const tile &t1, const tile &t2) const {
    return t1.distance(t2);
  }
};

class tile_neighbors_fn {
public:
  std::vector<tile> operator()(const tile &t) const {
    return {
        {t.x - 1, t.y},
        {t.x + 1, t.y},
        {t.x, t.y - 1},
        {t.x, t.y + 1},
    };
  }
};

using tile_pathfinder =
    astar::pathfinder<tile, tile_cost_fn, tile_heuristic_fn,
                      tile_neighbors_fn>;

int main(int argc, char **args) {
  tile start{0, 0};
  tile goal{10, 10};

  tile_pathfinder pathfinder{tile_cost_fn{},
                             tile_heuristic_fn{},
                             tile_neighbors_fn{}};

  auto path = pathfinder.search(start, goal);
  for (const auto &tile : path.value()) {
    std::cout << tile.x << ", " << tile.y << std::endl;
  }

  return 0;
}