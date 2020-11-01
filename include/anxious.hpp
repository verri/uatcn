#pragma once

#include <uat/agent.hpp>
#include <uat/airspace.hpp>
#include <uat/permit.hpp>
#include <uat/type.hpp>

#include <cstdio>
#include <limits>
#include <unordered_set>

class Anxious
{
public:
  Anxious(const uat::airspace&, int);

  auto bid_phase(uat::uint_t, uat::bid_fn, uat::permit_public_status_fn, int) -> void;

  auto ask_phase(uat::uint_t, uat::ask_fn, uat::permit_public_status_fn, int) -> void;

  auto on_bought(const uat::region&, uat::uint_t, uat::value_t) -> void;

  auto stop(uat::uint_t, int) -> bool;

private:
  uat::mission_t mission_;
  uat::value_t fundamental_;
  std::unordered_set<uat::permit> keep_, onsale_;
  std::size_t path_size_ = std::numeric_limits<std::size_t>::max();
};

static_assert(uat::agent_traits<Anxious>::has_mb_bid_phase);
static_assert(uat::agent_traits<Anxious>::has_mb_ask_phase);
static_assert(uat::agent_traits<Anxious>::has_mb_on_bought);
static_assert(!uat::agent_traits<Anxious>::has_mb_on_sold);
