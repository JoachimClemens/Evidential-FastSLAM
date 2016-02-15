/*
 * Software License Agreement (BSD License)
 *
 *  Evidential FastSLAM - An evidential approach to SLAM
 *  Copyright (c) 2013-2016, Joachim Clemens, Thomas Reineking, Tobias Kluth
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Evidential FastSLAM nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EFS_MAPCACHE_H_
#define EFS_MAPCACHE_H_

#include <unordered_map>
#include <boost/unordered_map.hpp>

#include "efslam/utils/ConditionalMutex.h"


namespace efs {

/*
// Forward declarations
template<typename Map>	class MapCacheRead;
template<typename Map>	class MapCacheWrite;
*/

/**
 * Read makes the const access methods available and Write makes the non-const access methods available
 * It is advice to use the specialized variants only (e.g. set Read to false of you only need write acces),
 * because that makes the methods more efficient.
 * Be careful with write access and the non-const methods, because they may accidently allocate
 * new cells in the map.
 */
template<typename Map, bool Read = true, bool Write = true, bool ThreadSafe = false>
class MapCache {
public:
	static constexpr int	Dimension 	= Map::Dimension;
	using					WorldPoint	= typename Map::WorldPoint;
	using					MapPoint	= typename Map::MapPoint;;
	using					CellType	= typename Map::CellType;
	template<typename CellTypeOut = CellType>
	using					CellList	= typename Map::template CellList<CellTypeOut>;

									MapCache( Map *map );
									MapCache( const Map *map );
									/* No conversion allowed
									MapCache( const MapCache<Map, true, true> &mc );
									MapCache( const MapCache<Map, true, false> &mc );
									MapCache( const MapCache<Map, false, true> &mc );
									*/

	inline 	MapPoint				world2map( const WorldPoint &p ) const		{	return m_mapConst->world2map( p );	}
	inline	WorldPoint				map2world( const MapPoint &p ) const		{	return m_mapConst->map2world( p );	}

	inline	const CellType&			operator[]( const WorldPoint &p ) const 	{ 	return cell( p ); }
	inline	CellType&				operator[]( const WorldPoint &p )			{ 	return cell( p ); }

	inline	const CellType&			operator[]( const MapPoint &p ) const		{ 	return cell( p ); }
	inline	CellType&				operator[]( const MapPoint &p )				{ 	return cell( p ); }

	inline	const CellType&			cell( const WorldPoint &p ) const			{ 	return cell( world2map( p ) ); }
	inline	CellType&				cell( const WorldPoint &p )					{ 	return cell( world2map( p ) ); }

	// These are the actual access methods
	inline	const CellType&			cell( const MapPoint &p ) const;
	inline	CellType&				cell( const MapPoint &p );

	// this does not accidently allocate new cells, that are not already in the map
	inline	const CellType&			cellOrUnknown( const WorldPoint &p ) const	{ 	return cell( world2map( p ) ); }
	inline	const CellType&			cellOrUnknown( const MapPoint &p ) const	{ 	return cell( p ); }


	template<typename CellTypeOut = CellType>
	inline	CellList<CellTypeOut>	cells() const								{	return m_mapConst->cells<CellTypeOut>(); }

	inline	uint32_t				prune();

	inline 	bool 					isInside( const MapPoint &p ) const 		{	return m_mapConst->isInside( p ); }
	inline 	bool 					isInside( const WorldPoint &p ) const 		{	return m_mapConst->isInside( p ); }

	inline 	void					clearCache();

	inline	const WorldPoint&		center() const								{	return m_mapConst->center(); 	}
	inline	const WorldPoint&		worldSize() const							{	return m_mapConst->worldSize();	}
	inline	const MapPoint&			mapSize() const								{	return m_mapConst->mapSize(); 	}
	inline	const world_t&			delta() const								{	return m_mapConst->delta();		}
	inline	size_t					bytes() const								{	return m_mapConst->bytes();		}

	/*
	friend MapCache<Map, true, false>;
	friend MapCache<Map, false, true>;
	friend MapCache<Map, true, true>;
	*/

protected:
	Map			*m_map;
	const Map	*m_mapConst;

	/*
			std::unordered_map<MapPoint, CellType*>			m_cacheWrite;
	mutable std::unordered_map<MapPoint, const CellType*>	m_cacheRead;
	*/
	// /*
			boost::unordered_map<MapPoint, CellType* >			m_cacheWrite;
	mutable boost::unordered_map<MapPoint, const CellType* >	m_cacheRead;
	// */
	mutable ConditionalMutex<ThreadSafe>					m_mutex;

};

// /*
template<typename Map, bool ThreadSafe = false>	using	MapCacheRead	= MapCache<Map, true, false, ThreadSafe>;
template<typename Map, bool ThreadSafe = false>	using	MapCacheWrite	= MapCache<Map, false, true, ThreadSafe>;
// */

/*
template<typename Map>
class MapCacheRead : public MapCache<Map, true, false> {
public:
	MapCacheRead( const Map *map ) : MapCache<Map, true, false>( map ) {}
};


template<typename Map>
class MapCacheWrite : public MapCache<Map, false, true> {
public:
	MapCacheWrite( Map *map ) : MapCache<Map, false, true>( map ) {}
};
*/

} /* namespace efs */


#include "MapCache.hpp"

#endif /* EFS_MAPCACHE_H_ */
