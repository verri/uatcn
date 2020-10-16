#include "hexgrid.hpp"

#include <algorithm>
#include <iterator>
#include <random>
#include <tuple>
#include <utility>

HexGrid::HexGrid(std::array<int, 3> dim) : dim_{dim}
{
  assert(dim_[0] > 1);
  assert(dim_[1] > 1);
  assert(dim_[2] > 1);
}

auto HexGrid::random_mission(int seed) const -> uat::mission_t
{
  std::mt19937 g(seed);

  std::uniform_int_distribution<int > rows{0, dim_[0] - 1};
  std::uniform_int_distribution<int> cols{0, dim_[1] - 1};

  const std::array<int, 2> from = {rows(g), cols(g)};
  auto to = from;

  do {
    to[0] = rows(g);
    to[1] = cols(g);
  } while (from == to);

  return {
    HexPermit{from[0], from[1], 0, dim_},
    HexPermit{to[0], to[1], 0, dim_}
  };
}

auto HexGrid::dimensions() const -> std::array<uint_t, 3u> {
  return {
    static_cast<uint_t>(dim_[0]),
    static_cast<uint_t>(dim_[1]),
    static_cast<uint_t>(dim_[2])
  };
}

auto HexPermit::neighbors() const -> std::vector<uat::slot>
{
  // see: https://www.redblobgames.com/grids/hexagons/#neighbors-offset
  constexpr int nei_map[2u][6u][2u] = {
    {
      {+1, 0}, {0, -1}, {-1, -1}, {-1, 0}, {-1, +1}, {0, +1}
    },
    {
      {+1, 0}, {+1, -1}, { 0, -1}, {-1,  0}, { 0, +1}, {+1, +1}
    },
  };

  std::vector<uat::slot> nei;
  nei.reserve(8);

  for (const auto [i, j] : nei_map[row_ & 1]) {
    if (row_ + i < 0 || row_ + i >= limits_[0])
      continue;
    if (col_ + j < 0 || col_ + j >= limits_[1])
      continue;
    nei.push_back(HexPermit{row_ + i, col_ + j, altitute_, limits_});
  }

  if (altitute_ > 0)
    nei.push_back(HexPermit{row_, col_, altitute_ - 1, limits_});

  if (altitute_ < limits_[2] - 1)
    nei.push_back(HexPermit{row_, col_, altitute_ + 1, limits_});

  return nei;
}

auto HexPermit::hash() const -> std::size_t
{
  return row_ * limits_[1] * limits_[2] + col_ * limits_[2] + altitute_;
}


auto HexPermit::operator==(const HexPermit& other) const -> bool
{
  return row_ == other.row_ && col_ == other.col_ && altitute_ == other.altitute_;
}

auto HexPermit::cube_coord() const -> std::array<int, 3u>
{
  const auto x = col_ - (row_ - (row_ & 1)) / 2;
  const auto z = row_;
  const auto y = - x - z;
  return {x, y, z};
}

auto HexPermit::distance(const HexPermit& other) const -> uint_t
{
  constexpr auto abs = [](auto x, auto y) constexpr { return x > y ? x - y : y - x; };

  const auto pos = cube_coord();
  const auto opos = other.cube_coord();

  return (abs(pos[0], opos[0]) + abs(pos[1], opos[1]) + abs(pos[2], opos[2])) / 2 +
    abs(altitute_, other.altitute_);
}

auto HexPermit::print(std::function<void(std::string_view, fmt::format_args)> f) const -> void
{
  f("{},{},{}", fmt::make_format_args(row_, col_, altitute_));
}
