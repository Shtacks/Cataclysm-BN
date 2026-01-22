#pragma once

#include <bitset>
#include <map>
#include <set>
#include <vector>

#include "coordinates.h"
#include "cube_direction.h"

class overmap;

namespace plumbing_grid
{

using connection_bitset = std::bitset<six_cardinal_directions.size()>;
using connection_map = std::map<tripoint_om_omt, connection_bitset>;

auto connections_for( overmap &om ) -> connection_map &;
auto connections_for( const overmap &om ) -> const connection_map &;

auto grid_at( const tripoint_abs_omt &p ) -> std::set<tripoint_abs_omt>;
auto grid_connectivity_at( const tripoint_abs_omt &p ) -> std::vector<tripoint_rel_omt>;
auto add_grid_connection( const tripoint_abs_omt &lhs, const tripoint_abs_omt &rhs ) -> bool;
auto remove_grid_connection( const tripoint_abs_omt &lhs, const tripoint_abs_omt &rhs ) -> bool;
auto clear() -> void;

} // namespace plumbing_grid
