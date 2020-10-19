#pragma once

#include <type_traits>
#include <uat/airspace.hpp>
#include <uat/permit.hpp>
#include <uat/type.hpp>

class HexGrid
{
public:
  explicit HexGrid(std::array<int, 3>);

  auto random_mission(int) const -> uat::mission_t;

  auto iterate(uat::region_fn) const -> void;

private:
  std::array<int, 3> dim_;
};

// odd-r representation
// consult: https://www.redblobgames.com/grids/hexagons/
class HexRegion
{
  friend class HexGrid;

public:
  auto adjacent_regions() const -> std::vector<uat::region>;

  auto hash() const -> std::size_t;

  auto operator==(const HexRegion&) const -> bool;

  auto distance(const HexRegion&) const -> uat::uint_t;

  auto print(std::function<void(std::string_view, fmt::format_args)>) const -> void;

  auto altitude() const { return altitude_; }

private:
  HexRegion(int row, int col, int alt, std::array<int, 3> limits) : row_{row}, col_{col}, altitude_{alt}, limits_{limits} {}

  auto cube_coord() const -> std::array<int, 3u>;

  int row_, col_;
  int altitude_;
  std::array<int, 3> limits_;
};
