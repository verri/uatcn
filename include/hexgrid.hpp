#pragma once

#include <type_traits>
#include <uat/type.hpp>
#include <uat/airspace.hpp>
#include <uat/slot.hpp>

using uint_t = uat::uint_t;

class HexGrid
{
public:
  explicit HexGrid(std::array<int, 3>);

  auto random_mission(int) const -> uat::mission_t;

  auto dimensions() const -> std::array<uint_t, 3u>;

private:
  std::array<int, 3> dim_;
};

// odd-r representation
// consult: https://www.redblobgames.com/grids/hexagons/
class HexPermit
{
  friend class HexGrid;

public:
  auto neighbors() const -> std::vector<uat::slot>;

  auto hash() const -> std::size_t;

  auto operator==(const HexPermit&) const -> bool;

  auto distance(const HexPermit&) const -> uint_t;

  auto print(std::function<void(std::string_view, fmt::format_args)>) const -> void;

private:
  HexPermit(int row, int col, int alt, std::array<int, 3> limits) :
    row_{row}, col_{col}, altitute_{alt}, limits_{limits} {}

  auto cube_coord() const -> std::array<int, 3u>;

  int row_, col_;
  int altitute_;
  std::array<int, 3> limits_;
};
