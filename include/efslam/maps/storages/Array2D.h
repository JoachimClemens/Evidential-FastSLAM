/*
 * Software License Agreement (BSD License)
 *
 *  Evidential FastSLAM - An evidential approach to SLAM
 *  Copyright (c) 2013-2016, Joachim Clemens, Thomas Reineking, Tobias Kluth
 *  All rights reserved.
 *
 *  This file is partially based on the GMapping Array2D class
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

#ifndef EFS_ARRAY2D_H_
#define EFS_ARRAY2D_H_

#include <iostream>
#include <list>

#include <assert.h>

#include "efslam/utils/Point.h"
#include "efslam/maps/AccessState.h"


namespace efs {

template<class Cell, const bool debug = false>
class Array2D {
public:
	static constexpr int	Dimension 	= 2;
	template<typename CellOut = Cell>
	using					CellList	= std::list< std::pair<Point2m, const CellOut &> >;

								Array2D( const Point2m &size );
								Array2D( map_t xsize = 0, map_t ysize = 0 );
								Array2D( const Array2D<Cell, debug> &other );
								Array2D( Array2D<Cell, debug> &&other );
								~Array2D();

			void 				clear();
			void 				resize( map_t xmin, map_t ymin, map_t xmax, map_t ymax );

			Array2D&			operator=( const Array2D &other );
			Array2D&			operator=( Array2D &&other );

	inline 	const Cell& 		cell( map_t x, map_t y ) const;
	inline 	Cell& 				cell( map_t x, map_t y );
	inline 	const Cell&			cell( const Point2m& p ) const 		{	return cell( (map_t) p[0], (map_t) p[1] );	}
	inline 	Cell& 				cell( const Point2m& p ) 			{	return cell( (map_t) p[0], (map_t) p[1] );	}

	template<typename CellOut = Cell>
	inline 	CellList<CellOut>	cells( const Point2m &offset = Point2m::Zero() ) const;

	inline	constexpr uint32_t	prune() const 						{ 	return 0; } // Array2D can't be pruned

	inline	size_t				bytes() const						{	return sizeof(*this) + sizeof(Cell*) * m_size[0] + sizeof(Cell) * m_size[0] * m_size[1]; }

	inline 	bool 				isInside( map_t x, map_t y ) const;
	inline 	bool 				isInside( const Point2m &p ) const 	{	return isInside( (map_t) p[0], (map_t) p[1] );	}

	inline 	AccessibilityState 	cellState( map_t x, map_t y ) const {	return (AccessibilityState) (isInside( x, y ) ? (AS_INSIDE | AS_ALLOCATED) : AS_OUTSIDE);	}
	inline 	AccessibilityState	cellState( const Point2m &p ) const	{	return cellState( (map_t) p[0], (map_t) p[1] );	}

	inline 	int 				getXSize() const 					{	return m_size[0];		}
	inline 	int 				getYSize() const 					{	return m_size[1];		}
	inline 	Point2m				size() const						{ 	return m_size; 			}
	inline	Point2m				patchSize() const					{ 	return size(); 			}
	inline	Point2m				numPatches() const					{ 	return Point3m::Ones();	}

	//inline Cell** 			cells() 										{	return m_cells;	}

protected:
	Cell 	**m_cells;
	Point2m	m_size;
};

} /* namespace efs */

#include "Array2D.hpp"

#endif /* EFS_ARRAY_2D_H_ */

