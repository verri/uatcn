#pragma once

#include <uat/slot.hpp>
#include <uat/agent.hpp>
#include <uat/type.hpp>

// TODO do not consider turn or climb
// We assume flight cost is negligible
// We assume two adjacent permits are selected per step (avoids collision)
auto astar(const uat::slot& from, const uat::slot& to, uat::uint_t tstart,
           uat::value_t bid_max_value, uat::status_t& status, int seed) -> std::vector<uat::tslot>;
