#include "plumbing_grid.h"

#include <algorithm>
#include <cstdlib>
#include <queue>
#include <ranges>

#include "coordinate_conversions.h"
#include "debug.h"
#include "overmap.h"
#include "overmapbuffer.h"

namespace
{

using connection_store = std::map<point_abs_om, plumbing_grid::connection_map>;

auto plumbing_grid_store() -> connection_store &
{
    static auto store = connection_store{};
    return store;
}

auto empty_connections() -> const plumbing_grid::connection_map &
{
    static const auto empty = plumbing_grid::connection_map{};
    return empty;
}

auto empty_bitset() -> const plumbing_grid::connection_bitset &
{
    static const auto empty = plumbing_grid::connection_bitset{};
    return empty;
}

auto connection_bitset_at( const tripoint_abs_omt &p ) -> const plumbing_grid::connection_bitset &
{
    const auto omc = overmap_buffer.get_om_global( p );
    const auto &connections = plumbing_grid::connections_for( *omc.om );
    const auto iter = connections.find( omc.local );
    if( iter == connections.end() ) {
        return empty_bitset();
    }
    return iter->second;
}

auto connection_bitset_at( overmap &om, const tripoint_om_omt &p ) -> plumbing_grid::connection_bitset &
{
    auto &connections = plumbing_grid::connections_for( om );
    return connections[p];
}

} // namespace

namespace plumbing_grid
{

auto connections_for( overmap &om ) -> connection_map &
{
    return plumbing_grid_store()[om.pos()];
}

auto connections_for( const overmap &om ) -> const connection_map &
{
    const auto &store = plumbing_grid_store();
    const auto iter = store.find( om.pos() );
    if( iter == store.end() ) {
        return empty_connections();
    }
    return iter->second;
}

auto grid_at( const tripoint_abs_omt &p ) -> std::set<tripoint_abs_omt>
{
    auto result = std::set<tripoint_abs_omt>{};
    auto open = std::queue<tripoint_abs_omt>{};
    open.emplace( p );

    while( !open.empty() ) {
        const auto &elem = open.front();
        result.emplace( elem );
        const auto &connections_bitset = connection_bitset_at( elem );
        std::ranges::for_each( std::views::iota( size_t{ 0 }, six_cardinal_directions.size() ),
        [&]( size_t i ) {
            if( connections_bitset.test( i ) ) {
                const auto other = elem + six_cardinal_directions[i];
                if( !result.contains( other ) ) {
                    open.emplace( other );
                }
            }
        } );
        open.pop();
    }

    return result;
}

auto grid_connectivity_at( const tripoint_abs_omt &p ) -> std::vector<tripoint_rel_omt>
{
    auto ret = std::vector<tripoint_rel_omt>{};
    ret.reserve( six_cardinal_directions.size() );

    const auto &connections_bitset = connection_bitset_at( p );
    std::ranges::for_each( std::views::iota( size_t{ 0 }, six_cardinal_directions.size() ),
    [&]( size_t i ) {
        if( connections_bitset.test( i ) ) {
            ret.emplace_back( six_cardinal_directions[i] );
        }
    } );

    return ret;
}

auto add_grid_connection( const tripoint_abs_omt &lhs, const tripoint_abs_omt &rhs ) -> bool
{
    if( project_to<coords::om>( lhs ).xy() != project_to<coords::om>( rhs ).xy() ) {
        debugmsg( "Connecting plumbing grids on different overmaps is not supported yet" );
        return false;
    }

    const auto coord_diff = rhs - lhs;
    if( std::abs( coord_diff.x() ) + std::abs( coord_diff.y() ) + std::abs( coord_diff.z() ) != 1 ) {
        debugmsg( "Tried to connect non-orthogonally adjacent points" );
        return false;
    }

    auto lhs_omc = overmap_buffer.get_om_global( lhs );
    auto rhs_omc = overmap_buffer.get_om_global( rhs );

    const auto lhs_iter = std::ranges::find( six_cardinal_directions, coord_diff.raw() );
    const auto rhs_iter = std::ranges::find( six_cardinal_directions, -coord_diff.raw() );

    auto lhs_i = static_cast<size_t>( std::distance( six_cardinal_directions.begin(), lhs_iter ) );
    auto rhs_i = static_cast<size_t>( std::distance( six_cardinal_directions.begin(), rhs_iter ) );

    auto &lhs_bitset = connection_bitset_at( *lhs_omc.om, lhs_omc.local );
    auto &rhs_bitset = connection_bitset_at( *rhs_omc.om, rhs_omc.local );

    if( lhs_bitset[lhs_i] && rhs_bitset[rhs_i] ) {
        debugmsg( "Tried to connect to plumbing grid two points that are connected to each other" );
        return false;
    }

    lhs_bitset[lhs_i] = true;
    rhs_bitset[rhs_i] = true;
    return true;
}

auto remove_grid_connection( const tripoint_abs_omt &lhs, const tripoint_abs_omt &rhs ) -> bool
{
    const auto coord_diff = rhs - lhs;
    if( std::abs( coord_diff.x() ) + std::abs( coord_diff.y() ) + std::abs( coord_diff.z() ) != 1 ) {
        debugmsg( "Tried to disconnect non-orthogonally adjacent points" );
        return false;
    }

    auto lhs_omc = overmap_buffer.get_om_global( lhs );
    auto rhs_omc = overmap_buffer.get_om_global( rhs );

    const auto lhs_iter = std::ranges::find( six_cardinal_directions, coord_diff.raw() );
    const auto rhs_iter = std::ranges::find( six_cardinal_directions, -coord_diff.raw() );

    auto lhs_i = static_cast<size_t>( std::distance( six_cardinal_directions.begin(), lhs_iter ) );
    auto rhs_i = static_cast<size_t>( std::distance( six_cardinal_directions.begin(), rhs_iter ) );

    auto &lhs_bitset = connection_bitset_at( *lhs_omc.om, lhs_omc.local );
    auto &rhs_bitset = connection_bitset_at( *rhs_omc.om, rhs_omc.local );

    if( !lhs_bitset[lhs_i] && !rhs_bitset[rhs_i] ) {
        debugmsg( "Tried to disconnect from plumbing grid two points with no connection to each other" );
        return false;
    }

    lhs_bitset[lhs_i] = false;
    rhs_bitset[rhs_i] = false;
    return true;
}

auto clear() -> void
{
    plumbing_grid_store().clear();
}

} // namespace plumbing_grid
