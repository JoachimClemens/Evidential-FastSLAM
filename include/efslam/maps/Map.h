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

#ifndef EFS_MAP_H_
#define EFS_MAP_H_

#include "AccessState.h"

#include "efslam/utils/Point.h"
#include "efslam/utils/Convenience.h"

namespace efs {

template<class Cell, class Storage>
class Map {
public:
	static constexpr int	Dimension 	= Storage::Dimension;
	using					WorldPoint	= Pointw<Dimension>;
	using					MapPoint	= Pointm<Dimension>;
	using					CellType	= Cell;
	template<typename CellOut = Cell>
	using					CellList	= typename Storage::template CellList<CellOut>;

								Map( const MapPoint &size = MapPoint::Zero(), world_t delta = 1.0 );
								Map( const WorldPoint &center, const WorldPoint &size, world_t delta );
								Map( const WorldPoint &center, const WorldPoint &min, const WorldPoint &max, world_t delta );
	virtual						~Map();

	inline	void				clear();

	inline 	MapPoint			world2map( const WorldPoint &p ) const;
	inline	WorldPoint			map2world( const MapPoint &p ) const;

	inline	const Cell&			operator[]( const WorldPoint &p ) const 	{ 	return cell( p ); }
	inline	Cell&				operator[]( const WorldPoint &p )			{ 	return cell( p ); }

	inline	const Cell&			operator[]( const MapPoint &p ) const		{ 	return cell( p ); }
	inline	Cell&				operator[]( const MapPoint &p )				{ 	return cell( p ); }

	inline	const Cell&			cell( const WorldPoint &p ) const			{ 	return cell( world2map( p ) ); }
	inline	Cell&				cell( const WorldPoint &p )					{ 	return cell( world2map( p ) ); }

	inline	const Cell&			cell( const MapPoint &p ) const				{ 	return m_storage.cell( p ); }
	inline	Cell&				cell( const MapPoint &p )					{ 	return m_storage.cell( p ); }

	// this does not accidently allocate new cells, that are not already in the map
	inline	const Cell&			cellOrUnknown( const WorldPoint &p ) const	{ 	return cell( p ); }
	inline	const Cell&			cellOrUnknown( const MapPoint &p ) const	{ 	return cell( p ); }

	template<typename CellOut = Cell>
	inline	CellList<CellOut>	cells() const								{	return m_storage.cells<CellOut>(); }

	inline	uint32_t			prune()										{	return m_storage.prune(); }

	inline 	bool 				isInside( const MapPoint &p ) const 		{	return m_storage.isInside( p ); }
	inline 	bool 				isInside( const WorldPoint &p ) const 		{	return m_storage.isInside( world2map( p ) ); }

	// for shared storages, the size of the shared parts is given relative to the number of shares
	inline	size_t				bytes() const;

	inline	MapPoint			patchSize() const							{	return m_storage.patchSize();	}
	inline	MapPoint			numPatches() const							{	return m_storage.numPatches();	}

			void				load( std::istream &f );
			void				save( std::ostream &f ) const;

	/* TODO
	DoubleArray2D* toDoubleArray() const;
	Map<double, DoubleArray2D, false>* toDoubleMap() const;

	To storage?
	void load( filename );
	void save( filename ) const;
	 */


private:
	void	initMembers( world_t delta, const WorldPoint *center = nullptr );

	MapPoint	m_mapSize;
	WorldPoint	m_worldSize,
				m_center;
	world_t		m_delta;
	Storage 	m_storage;

	//static WorldPoint sm_point5;

public:
	GETTER( center );
	GETTER( worldSize );
	GETTER( mapSize );
	GETTER( delta );
	//GETTER( storage );
};

} /* namespace efs */

#include "Map.hpp"

#endif /* EFS_MAP_H_ */
