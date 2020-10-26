#include "anxious.hpp"
#include "astar.hpp"

#include <cmath>
#include <cool/compose.hpp>
#include <cool/indices.hpp>
#include <jules/base/numeric.hpp>
#include <cassert>
#include <cstdio>
#include <iterator>
#include <limits>
#include <random>
#include <unordered_set>
#include <utility>
#include <variant>

using namespace uat;

Anxious::Anxious(const airspace& space, [[maybe_unused]] uint_t T, int seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<value_t> f{1.0, 2.0};
  fundamental_ = f(rng);
  mission_ = space.random_mission(rng());
  assert(T > mission_.length());
}

auto Anxious::bid_phase(uint_t t, uat::bid_fn bid, uat::permit_public_status_fn status, int seed) -> void
{
  std::mt19937 rng(seed);
  onsale_ = std::exchange(keep_, {});

  const auto path = astar(mission_.from, mission_.to, t + 1, fundamental_, status, rng());

  if (path.size() == 0) {
    path_size_ = std::numeric_limits<std::size_t>::max();
    return;
  }

  path_size_ = path.size();
  for (auto& permit : path) {
    using namespace permit_public_status;
    const auto stat = status(permit.location(), permit.time());
    std::visit(cool::compose{
                 [](unavailable) { assert(false); },
                 [&](owned) {
                   onsale_.erase(permit);
                   keep_.insert(std::move(permit));
                 },
                 [&](available) { bid(permit.location(), permit.time(), fundamental_); },
               },
               stat);
  }
}

auto Anxious::ask_phase(uint_t, ask_fn ask, permit_public_status_fn, int) -> void
{
  for (const auto& permit : onsale_)
    ask(permit.location(), permit.time(), 0.0);
  onsale_.clear();
}

auto Anxious::on_bought(const region& s, uint_t t, value_t) -> void { keep_.insert({s, t}); }

auto Anxious::stop(uint_t, int) -> bool
{
  if (keep_.size() == path_size_)
    return true;
  fundamental_ *= 2.0;
  return false;
}
