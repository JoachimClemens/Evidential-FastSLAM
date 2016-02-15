/*
 * Software License Agreement (BSD License)
 *
 *  Evidential FastSLAM - An evidential approach to SLAM
 *  Copyright (c) 2013-2016, Joachim Clemens, Thomas Reineking, Tobias Kluth
 *  All rights reserved.
 *
 *  This file is partially based on the GMapping HArray2D class
 *  Copyright (c) 2004-2007, Giorgio Grisetti, Cyrill Stachniss, Wolfram Burgard
 *  Originally licensed under the Creative Commons (Attribution-NonCommercial-ShareAlike).
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

#include "HStorageHelper.h"

#include "efslam/utils/PointIterator.h"

namespace efs {


template<typename Cell, typename Storage, int Magnitude>
const Cell HierarchicalStorage<Cell, Storage, Magnitude>::sm_unknown = Cell(0);


template<typename Cell, typename Storage, int Magnitude>
HierarchicalStorage<Cell, Storage, Magnitude>::HierarchicalStorage( const PointNm &size ) {
	size_t vectorSize = 1;

	for( int i = 0; i < Dimension; i++ ) {
	    if( ((size[i] >> Magnitude) << Magnitude) != size[i] ) // have to round up?
	    	m_size[i] 	= (size[i] >> Magnitude) + 1;
	    else
	    	m_size[i] 	= (size[i] >> Magnitude);
		m_totalSize[i]	= m_size[i] << Magnitude;
		m_patchSize[i]	= 1 << Magnitude;
		vectorSize		*= m_size[i];
	}

	m_patches.resize( vectorSize );
}


template<typename Cell, typename Storage, int Magnitude>
HierarchicalStorage<Cell, Storage, Magnitude>::HierarchicalStorage( const HierarchicalStorage<Cell, Storage, Magnitude> &hs ) :
	m_size( hs.m_size ),
	m_totalSize( hs.m_totalSize ),
	m_patchSize( hs.m_patchSize ),
	m_patches( hs.m_patches )
{
	// Nothing else to do here, std::vector copy constructor and autoptrs should handel everything for us
}


template<typename Cell, typename Storage, int Magnitude>
HierarchicalStorage<Cell, Storage, Magnitude>::~HierarchicalStorage() {
	// Nothing to do here
}


template<typename Cell, typename Storage, int Magnitude>
void
HierarchicalStorage<Cell, Storage, Magnitude>::clear() {
	for( auto &patch : m_patches )
		patch = nullptr;
}


template<typename Cell, typename Storage, int Magnitude>
HierarchicalStorage<Cell, Storage, Magnitude>&
HierarchicalStorage<Cell, Storage, Magnitude>::operator=( const HierarchicalStorage<Cell, Storage, Magnitude> &hs ) {
	if( this != &hs ) {
		m_size 		= hs.m_size;
		m_totalSize	= hs.m_totalSize;
		m_patchSize	= hs.m_patchSize;
		m_patches	= hs.m_patches;
	}
	return *this;
}


template<typename Cell, typename Storage, int Magnitude>
Cell&
HierarchicalStorage<Cell, Storage, Magnitude>::cell( const PointNm &p ) {
	//std::cout << p.transpose() << std::endl;
	assert( isInside( p ) );

	size_t patchIdx = HierarchicalStorageHelper<Dimension>::patchIndex( p, m_size, Magnitude );

	if( !m_patches[patchIdx] )
		m_patches[patchIdx] = shared_unique_ptr<Storage>( new Storage( m_patchSize ) );

	//std::cout << m_patches[patchIdx].use_count() << std::endl;

	// Replicate this patch if it is shared with at least one other HStorage
	if( !m_patches[patchIdx].unique() )
		m_patches[patchIdx].make_unique();

	//assert( m_patches[patchIdx].unique() );

	return m_patches[patchIdx]->cell( HierarchicalStorageHelper<Dimension>::cellIndex( p, Magnitude ) );
}


template<typename Cell, typename Storage, int Magnitude>
const Cell&
HierarchicalStorage<Cell, Storage, Magnitude>::cell( const PointNm &p ) const {
	//std::cout << "cell const\n";
	assert( isInside( p ) );
	size_t patchIdx = HierarchicalStorageHelper<Dimension>::patchIndex( p, m_size, Magnitude );

	if( !m_patches[patchIdx] )
		return sm_unknown;

	//assert( isPatchAllocated( p ) );

	return m_patches[patchIdx]->cell( HierarchicalStorageHelper<Dimension>::cellIndex( p, Magnitude ) );
}


template<typename Cell, typename Storage, int Magnitude>
template<typename CellOut>
typename HierarchicalStorage<Cell, Storage, Magnitude>::template CellList<CellOut>
HierarchicalStorage<Cell, Storage, Magnitude>::cells() const {
	CellList<CellOut>	res;

	for( PointIterator<Dimension> patchIter( PointNm::Zero(), m_totalSize, m_patchSize ); patchIter; patchIter++ ) {
		size_t patchIdx = HierarchicalStorageHelper<Dimension>::patchIndex( *patchIter, m_size, Magnitude );
		if( m_patches[patchIdx] ) {
			// Append patch's cells to result
			res.splice( res.end(), m_patches[patchIdx]->cells<CellOut>( *patchIter ) );
		}
	}

	return res;
}


template<typename Cell, typename Storage, int Magnitude>
uint32_t
HierarchicalStorage<Cell, Storage, Magnitude>::prune() {
	uint32_t count = 0;

	for( auto &patch : m_patches )
		if( patch ) {
			patch.lock();
			count += patch->prune();
			patch.unlock();
		}

	return count;
}


template<typename Cell, typename Storage, int Magnitude>
size_t
HierarchicalStorage<Cell, Storage, Magnitude>::bytes() const {
	size_t bytes = sizeof( *this ) + sizeof( shared_unique_ptr<Storage> ) * m_patches.size();

	for( auto &patch : m_patches )
		if( patch )
			bytes += patch->bytes() / patch.use_count();

	return bytes;
}


template<typename Cell, typename Storage, int Magnitude>
AccessibilityState
HierarchicalStorage<Cell, Storage, Magnitude>::cellState( const PointNm& p ) const {
	if( isInside( p ) )	{
		if( isAllocated( p ) )
			return (AccessibilityState) ((int) AS_INSIDE | (int) AS_ALLOCATED);
		else
			return AS_INSIDE;
	}
	return AS_OUTSIDE;
}


template<typename Cell, typename Storage, int Magnitude>
bool
HierarchicalStorage<Cell, Storage, Magnitude>::isInside( const PointNm& p ) const {
	//std::cout << m_totalSize.transpose() << std::endl;
	for( int i = 0; i < Dimension; i++ )
		if( p[i] < 0 || p[i] >= m_totalSize[i] )
			return false;
	return true;
}


template<typename Cell, typename Storage, int Magnitude>
bool
HierarchicalStorage<Cell, Storage, Magnitude>::isAllocated( const PointNm &p ) const {
	size_t patchIdx = HierarchicalStorageHelper<Dimension>::patchIndex( p, m_size, Magnitude );

	if( !m_patches[patchIdx] )
		return false;

	return m_patches[patchIdx]->isAllocated( HierarchicalStorageHelper<Dimension>::cellIndex( p, Magnitude ) );
}



template<typename Cell, typename Storage, int Magnitude>
bool
HierarchicalStorage<Cell, Storage, Magnitude>::isPatchAllocated( const PointNm &p ) const {
	size_t patchIdx = HierarchicalStorageHelper<Dimension>::patchIndex( p, m_size, Magnitude );
	return m_patches[patchIdx] != 0;
}



} /* namespace efs */

