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
  std::uniform_real_distribution<value_t> f{0.5, 1.0};
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

  for (const auto& [slot, t] : path) {
    using namespace permit_public_status;
    std::visit(cool::compose{
                 [](unavailable) { assert(false); },
                 [&, slot = slot, t = t](owned) {
                   onsale_.erase({slot, t});
                   keep_.emplace(std::move(slot), t);
                 },
                 [&, slot = slot, t = t](available) { bid(std::move(slot), t, fundamental_); },
               },
               status(slot, t));
  }
}

auto Anxious::ask_phase(uint_t, ask_fn ask, permit_public_status_fn, int) -> void
{
  for (const auto& position : onsale_)
    ask(position.location(), position.time(), 0.0);
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
