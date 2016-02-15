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

namespace efs {

template<typename Map, bool Read, bool Write, bool ThreadSafe>
MapCache<Map, Read, Write, ThreadSafe>::MapCache( Map *map ) {
	m_mapConst = m_map = map;
}


template<typename Map, bool Read, bool Write, bool ThreadSafe>
MapCache<Map, Read, Write, ThreadSafe>::MapCache( const Map *map ) {
	static_assert( !Write, "For write access, a non-const map pointer is needed." );

	m_map 		= NULL;
	m_mapConst	= map;
}

/*
template<typename Map, bool Read, bool Write, bool ThreadSafe>
MapCache<Map, Read, Write, ThreadSafe>::MapCache( const MapCache<Map, true, true> &mc ) :
	m_map( mc.m_map ),
	m_mapConst( mc.m_mapConst ),
	m_cacheWrite( mc.m_cacheWrite ),
	m_cacheRead( mc.m_cacheRead )
{
}


template<typename Map, bool Read, bool Write, bool ThreadSafe>
MapCache<Map, Read, Write, ThreadSafe>::MapCache( const MapCache<Map, true, false> &mc ) :
	m_map( mc.m_map ),
	m_mapConst( mc.m_mapConst ),
	m_cacheWrite( mc.m_cacheWrite ),
	m_cacheRead( mc.m_cacheRead )
{
	static_assert( Read, "Copying a read only map requires write access." );
}


template<typename Map, bool Read, bool Write, bool ThreadSafe>
MapCache<Map, Read, Write, ThreadSafe>::MapCache( const MapCache<Map, false, true> &mc ) :
	m_map( mc.m_map ),
	m_mapConst( mc.m_mapConst ),
	m_cacheWrite( mc.m_cacheWrite ),
	m_cacheRead( mc.m_cacheRead )
{
	static_assert( Write, "Copying a write only map requires write access." );
}
*/


template<typename Map, bool Read, bool Write, bool ThreadSafe>
const typename MapCache<Map, Read, Write, ThreadSafe>::CellType&
MapCache<Map, Read, Write, ThreadSafe>::cell( const MapPoint &p ) const	{
	static_assert( Read, "Read access is needed to call this method." );

	std::lock_guard< ConditionalMutex<ThreadSafe> > guard( m_mutex );

	if( Write && !m_cacheWrite.empty() ) {
		// Already in write cache?
		const auto iter = m_cacheWrite.find( p );
		if( iter != m_cacheWrite.end() )
			return *iter->second;
	}

	// Already in read cache?
	const auto iter = m_cacheRead.find( p );
	if( iter != m_cacheRead.end() )
		return *iter->second;

	// Else, get it from map and save it to read cache (Not to write cache in any case,
	// because the cell may be unknown/unallocated and can't be changed later)
	const CellType *cell = &m_mapConst->cellOrUnknown( p );
	m_cacheRead.emplace( p, cell );

	return *cell;
}


template<typename Map, bool Read, bool Write, bool ThreadSafe>
typename MapCache<Map, Read, Write, ThreadSafe>::CellType&
MapCache<Map, Read, Write, ThreadSafe>::cell( const MapPoint &p ) {
	static_assert( Write, "Write access is needed to call this method." );
	//assert( m_map );

	if( Read && !m_cacheRead.empty() )
		m_cacheRead.clear();	// Invalidate read cache. It may contain unknown/unallocated cells that can't be changed.

	// Already in cache?
	auto iter = m_cacheWrite.find( p );
	if( iter != m_cacheWrite.end() )
		return *iter->second;

	// Else, get it from map and save it to write cache
	CellType *cell = &m_map->cell( p );
	m_cacheWrite.emplace( p, cell );
	return *cell;
}


template<typename Map, bool Read, bool Write, bool ThreadSafe>
uint32_t
MapCache<Map, Read, Write, ThreadSafe>::prune() {
	static_assert( Write, "Read access is needed to call this method." );
	clearCache();
	return m_map->prune();
}


template<typename Map, bool Read, bool Write, bool ThreadSafe>
void
MapCache<Map, Read, Write, ThreadSafe>::clearCache() {
	m_cacheWrite.clear();
	m_cacheRead.clear();
}



} /* namespace efs */
