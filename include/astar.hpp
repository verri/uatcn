#pragma once

#include <uat/agent.hpp>
#include <uat/permit.hpp>
#include <uat/type.hpp>

// TODO do not consider turn or climb
// We assume flight cost is negligible
// We assume two adjacent permits are selected per step (avoids collision)
auto astar(const uat::region& from, const uat::region& to, uat::uint_t tstart, uat::value_t bid_max_value,
           uat::permit_public_status_fn& status, int seed) -> std::vector<uat::permit>;
