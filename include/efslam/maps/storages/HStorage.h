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

#ifndef EFS_HSTORAGE_H_
#define EFS_HSTORAGE_H_

#include <vector>

#include "efslam/utils/shareduniqueptr.h"
#include "efslam/utils/Point.h"


namespace efs {


/**
 * Meta-Storage, that devides the map into equal-sized patches,
 * that are represented by the actual storage and hold as smart-pointers
 * to save memory when the maps are copied during resampling step.
 * The actual storages a replicated on demand.
 */
template<typename Cell, typename Storage, int Magnitude = 5>	// patches will have the size 2^Magnitude
class HierarchicalStorage {
public:
	static constexpr int	Dimension	= Storage::Dimension;
	using					PointNm		= Pointm<Dimension>;
	template<typename CellOut = Cell>
	using					CellList	= typename Storage::template CellList<CellOut>;

								HierarchicalStorage( const PointNm &size );
								HierarchicalStorage( const HierarchicalStorage<Cell, Storage, Magnitude> &hs );
								~HierarchicalStorage();

	inline	void				clear();

	inline	HierarchicalStorage<Cell, Storage, Magnitude>&
								operator=( const HierarchicalStorage<Cell, Storage, Magnitude> &hs );

	inline	Cell&				cell( const PointNm &p );
	inline	const Cell&			cell( const PointNm &p ) const;

	template<typename CellOut = Cell>
	inline	CellList<CellOut>	cells() const;

	inline	uint32_t			prune();

	// the size of the shared parts is given relative to the number of shares
	inline	size_t				bytes() const;

	inline 	AccessibilityState 	cellState( const PointNm& p ) const;
	inline 	bool 				isInside( const PointNm& p ) const;
	inline 	bool 				isAllocated( const PointNm &p ) const;
	inline 	bool 				isPatchAllocated( const PointNm &p ) const;

	inline 	PointNm				size() const		{ return m_totalSize; 	}
	inline	PointNm				patchSize() const	{ return m_patchSize; 	}
	inline	PointNm				numPatches() const	{ return m_size;		}
	

	// TODO: If having memory problems, hold only the currently needed patches in memory

protected:
	PointNm		m_size,			// size of the hierarchical storage
				m_totalSize,	// total/usable size
				m_patchSize;	// size of one patch (always quadratic)

	std::vector< shared_unique_ptr<Storage> >	m_patches;

	static const Cell sm_unknown;
};


} /* namespace efs */

#include "HStorage.hpp"

#endif /* EFS_HSTORAGE_H_ */
