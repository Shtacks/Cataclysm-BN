#include "plumbing_grid.h"

#include <algorithm>
#include <cstdlib>
#include <optional>
#include <queue>
#include <ranges>

#include "calendar.h"
#include "coordinate_conversions.h"
#include "coordinates.h"
#include "debug.h"
#include "game_constants.h"
#include "item.h"
#include "mapbuffer.h"
#include "mapdata.h"
#include "memory_fast.h"
#include "overmap.h"
#include "overmapbuffer.h"
#include "point.h"
#include "submap.h"

namespace
{

using connection_store = std::map<point_abs_om, plumbing_grid::connection_map>;
static const auto furn_standing_tank_plumbed_str = furn_str_id( "f_standing_tank_plumbed" );

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

struct plumbing_tank_location {
    tripoint_abs_sm submap;
    point pos;
};

class plumbing_storage_grid
{
    private:
        std::vector<tripoint_abs_sm> submap_coords;
        std::vector<plumbing_tank_location> tank_locations;
        mutable std::optional<plumbing_grid::water_storage_stats> cached_stats;

        mapbuffer &mb;

    public:
        plumbing_storage_grid( const std::vector<tripoint_abs_sm> &global_submap_coords,
                               mapbuffer &buffer ) :
            submap_coords( global_submap_coords ),
            mb( buffer )
        {
            std::ranges::for_each( submap_coords, [&]( const tripoint_abs_sm &sm_coord ) {
                auto *sm = mb.lookup_submap( sm_coord );
                if( sm == nullptr ) {
                    return;
                }

                std::ranges::for_each( std::views::iota( 0, SEEX ), [&]( int x ) {
                    std::ranges::for_each( std::views::iota( 0, SEEY ), [&]( int y ) {
                        const auto pos = point( x, y );
                        if( sm->get_furn( pos ).id() == furn_standing_tank_plumbed_str ) {
                            tank_locations.emplace_back( plumbing_tank_location{ sm_coord, pos } );
                        }
                    } );
                } );
            } );
        }

        auto empty() const -> bool
        {
            return tank_locations.empty();
        }

        auto invalidate() -> void
        {
            cached_stats.reset();
        }

        auto get_stats() const -> plumbing_grid::water_storage_stats
        {
            if( cached_stats ) {
                return *cached_stats;
            }

            auto stats = plumbing_grid::water_storage_stats{};
            std::ranges::for_each( tank_locations, [&]( const plumbing_tank_location &loc ) {
                auto *sm = mb.lookup_submap( loc.submap );
                if( sm == nullptr ) {
                    return;
                }
                if( sm->get_furn( loc.pos ).id() != furn_standing_tank_plumbed_str ) {
                    return;
                }

                const auto &furn = sm->get_furn( loc.pos ).obj();
                stats.capacity += furn.keg_capacity;
                const auto &items = sm->get_items( loc.pos );
                std::ranges::for_each( items, [&]( const item *it ) {
                    if( it != nullptr && it->made_of( LIQUID ) ) {
                        stats.stored += it->volume();
                    }
                } );
            } );

            cached_stats = stats;
            return stats;
        }

        auto drain_to( const tripoint_abs_ms &target ) -> void
        {
            auto total_volume = 0_ml;
            auto liquid_type = itype_id{};
            auto has_liquid = false;

            std::ranges::for_each( tank_locations, [&]( const plumbing_tank_location &loc ) {
                auto *sm = mb.lookup_submap( loc.submap );
                if( sm == nullptr ) {
                    return;
                }
                if( sm->get_furn( loc.pos ).id() != furn_standing_tank_plumbed_str ) {
                    return;
                }

                auto &items = sm->get_items( loc.pos );
                std::ranges::for_each( items, [&]( const item *it ) {
                    if( it != nullptr && it->made_of( LIQUID ) ) {
                        if( !has_liquid ) {
                            liquid_type = it->typeId();
                            has_liquid = true;
                        }
                        total_volume += it->volume();
                    }
                } );
                items.clear();
            } );

            cached_stats.reset();

            if( !has_liquid || total_volume <= 0_ml ) {
                return;
            }

            tripoint_abs_sm target_sm;
            point_sm_ms target_pos;
            std::tie( target_sm, target_pos ) = project_remain<coords::sm>( target );
            auto *target_submap = mb.lookup_submap( target_sm );
            if( target_submap == nullptr ) {
                return;
            }

            auto liquid_item = item::spawn( liquid_type, calendar::turn );
            liquid_item->charges = liquid_item->charges_per_volume( total_volume );
            target_submap->get_items( target_pos.raw() ).clear();
            target_submap->get_items( target_pos.raw() ).push_back( std::move( liquid_item ) );
        }
};

class plumbing_grid_tracker
{
    private:
        std::map<tripoint_abs_sm, shared_ptr_fast<plumbing_storage_grid>> parent_storage_grids;
        mapbuffer &mb;

        auto make_storage_grid_at( const tripoint_abs_sm &sm_pos ) -> plumbing_storage_grid &
        {
            const auto overmap_positions = plumbing_grid::grid_at( project_to<coords::omt>( sm_pos ) );
            auto submap_positions = std::vector<tripoint_abs_sm>{};
            submap_positions.reserve( overmap_positions.size() * 4 );

            std::ranges::for_each( overmap_positions, [&]( const tripoint_abs_omt &omp ) {
                const auto base = project_to<coords::sm>( omp );
                submap_positions.emplace_back( base + point_zero );
                submap_positions.emplace_back( base + point_east );
                submap_positions.emplace_back( base + point_south );
                submap_positions.emplace_back( base + point_south_east );
            } );

            if( submap_positions.empty() ) {
                static auto empty_storage_grid = plumbing_storage_grid( {}, MAPBUFFER );
                return empty_storage_grid;
            }

            auto storage_grid = make_shared_fast<plumbing_storage_grid>( submap_positions, mb );
            std::ranges::for_each( submap_positions, [&]( const tripoint_abs_sm &smp ) {
                parent_storage_grids[smp] = storage_grid;
            } );

            return *parent_storage_grids[submap_positions.front()];
        }

    public:
        plumbing_grid_tracker() : plumbing_grid_tracker( MAPBUFFER ) {}

        explicit plumbing_grid_tracker( mapbuffer &buffer ) : mb( buffer ) {}

        auto storage_at( const tripoint_abs_omt &p ) -> plumbing_storage_grid &
        {
            const auto sm_pos = project_to<coords::sm>( p );
            const auto iter = parent_storage_grids.find( sm_pos );
            if( iter != parent_storage_grids.end() ) {
                return *iter->second;
            }

            return make_storage_grid_at( sm_pos );
        }

        auto invalidate_at( const tripoint_abs_ms &p ) -> void
        {
            const auto sm_pos = project_to<coords::sm>( p );
            const auto iter = parent_storage_grids.find( sm_pos );
            if( iter != parent_storage_grids.end() ) {
                iter->second->invalidate();
            }
        }

        auto rebuild_at( const tripoint_abs_ms &p ) -> void
        {
            const auto sm_pos = project_to<coords::sm>( p );
            make_storage_grid_at( sm_pos );
        }

        auto disconnect_tank_at( const tripoint_abs_ms &p ) -> void
        {
            storage_at( project_to<coords::omt>( p ) ).drain_to( p );
        }

        auto clear() -> void
        {
            parent_storage_grids.clear();
        }
};

auto get_plumbing_grid_tracker() -> plumbing_grid_tracker &
{
    static auto tracker = plumbing_grid_tracker{};
    return tracker;
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

auto water_storage_at( const tripoint_abs_omt &p ) -> water_storage_stats
{
    return get_plumbing_grid_tracker().storage_at( p ).get_stats();
}

auto on_contents_changed( const tripoint_abs_ms &p ) -> void
{
    get_plumbing_grid_tracker().invalidate_at( p );
}

auto on_structure_changed( const tripoint_abs_ms &p ) -> void
{
    get_plumbing_grid_tracker().rebuild_at( p );
}

auto disconnect_tank( const tripoint_abs_ms &p ) -> void
{
    get_plumbing_grid_tracker().disconnect_tank_at( p );
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
    on_structure_changed( project_to<coords::ms>( lhs ) );
    on_structure_changed( project_to<coords::ms>( rhs ) );
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
    on_structure_changed( project_to<coords::ms>( lhs ) );
    on_structure_changed( project_to<coords::ms>( rhs ) );
    return true;
}

auto clear() -> void
{
    plumbing_grid_store().clear();
    get_plumbing_grid_tracker().clear();
}

} // namespace plumbing_grid
