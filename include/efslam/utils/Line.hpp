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


// Declaration of specialized methods

template<>
inline void
Line<2>::bresenham( const Point2m &start, const Point2m &end );

template<>
inline void
Line<3>::bresenham( const Point3m &start, const Point3m &end );




template<int N>
Line<N>::Line( const Pointm<N> &start, const Pointm<N> &end ) {
	bresenham( start, end );
}


template<int N>
Line<N>::~Line() {
	// Nothing to do here
}


template<int N>
void
Line<N>::extend( const Pointm<N> &end ) {
	Pointm<N> start = this->back();
	this->pop_back(); // do not add the last point twice
	bresenham( start, end );
}


template<int N>
inline void
Line<N>::bresenham( const Pointm<N> &start, const Pointm<N> &end ) {
	static_assert( N != 2 && N != 3, "Specialization needed for this N." );
}


template<>
inline void
Line<2>::bresenham( const Point2m &start, const Point2m &end ) {
	// 2D Bresenham's algorithm according to https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm

	int	dx 	= abs( (int) end[0] - (int) start[0] ),
		dy 	= abs( (int) end[1] - (int) start[1] ),
		sx 	= (start[0] < end[0]) ? 1 : -1,
		sy 	= (start[1] < end[1]) ? 1 : -1,
		err	= dx - dy,
		e2;

	Point2m cur = start;

	// reserve some memory
	reserve( size() + sqrt( (end - start).squaredNorm() ) + 2 ); // Eigen norm() doesn't work for integer types, so calc sqrt separately

	while( true ) {
		push_back( cur );

		if( cur == end )
			break;

		e2 = 2*err;

		if( e2 > -dy ) {
			err 	-= dy;
			cur[0]	+= sx;
			if( cur == end ) {
				push_back( cur );
				break;
			}
		}

		if( e2 < dx ) {
			err 	+= dx;
			cur[1]	+= sy;
		}
	}
}


template<>
inline void
Line<3>::bresenham( const Point3m &start, const Point3m &end ) {
	/*
	 * 3D Bresenham's algorithm according to https://gist.githubusercontent.com/yamamushi/5823518/raw/6571ec0fa01d2e3378a9f5bdebdd9c41b176f4ed/bresenham3d
	 *
	 * A slightly modified version of the source found at
	 * http://www.ict.griffith.edu.au/anthony/info/graphics/bresenham.procs
	 * Provided by Anthony Thyssen, though he does not take credit for the original implementation
	 *
	 * It is highly likely that the original Author was Bob Pendelton, as referenced here
	 *
	 * ftp://ftp.isc.org/pub/usenet/comp.sources.unix/volume26/line3d
	 *
	 * line3d was dervied from DigitalLine.c published as "Digital Line Drawing"
	 * by Paul Heckbert from "Graphics Gems", Academic Press, 1990
	 *
	 * 3D modifications by Bob Pendleton. The original source code was in the public
	 * domain, the author of the 3D version places his modifications in the
	 * public domain as well.
	 *
	 * line3d uses Bresenham's algorithm to generate the 3 dimensional points on a
	 * line from (x1, y1, z1) to (x2, y2, z2)
	 */

	int dx 	= end[0] - start[0],
		dy 	= end[1] - start[1],
		dz 	= end[2] - start[2],
		sx 	= (dx < 0) ? -1 : 1,
		sy 	= (dy < 0) ? -1 : 1,
		sz 	= (dz < 0) ? -1 : 1,
		l	= abs( dx ),
		m	= abs( dy ),
		n	= abs( dz ),
		dx2	= l << 1,
		dy2	= m << 1,
		dz2	= n << 1,
		err1,
		err2,
		i;

	Point3m cur = start;

	// reserve some memory
	reserve( size() + sqrt( (end - start).squaredNorm() ) + 2 ); // Eigen norm() doesn't work for integer types, so calc sqrt separately

	if( (l >= m) && (l >= n) ) {
		err1 = dy2 - l;
		err2 = dz2 - l;

		for( i = 0; i < l; i++ ) {
			push_back( cur );

			if( err1 > 0 ) {
				cur[1]	+= sy;
				err1 	-= dx2;
			}

			if( err2 > 0 ) {
				cur[2]	+= sz;
				err2 	-= dx2;
			}

			err1 	+= dy2;
			err2 	+= dz2;
			cur[0]	+= sx;
		}
	} else if( (m >= l) && (m >= n) ) {
		err1 = dx2 - m;
		err2 = dz2 - m;

		for( i = 0; i < m; i++ ) {
			push_back( cur );

			if( err1 > 0 ) {
				cur[0]	+= sx;
				err1 	-= dy2;
			}

			if( err2 > 0 ) {
				cur[2]	+= sz;
				err2 	-= dy2;
			}

			err1	+= dx2;
			err2	+= dz2;
			cur[1]	+= sy;
		}
	} else {
		err1 = dy2 - n;
		err2 = dx2 - n;

		for( i = 0; i < n; i++ ) {
			push_back( cur );

			if( err1 > 0 ) {
				cur[1]	+= sy;
				err1	-= dz2;
			}

			if( err2 > 0 ) {
				cur[0]	+= sx;
				err2	-= dz2;
			}

			err1	+= dy2;
			err2	+= dx2;
			cur[2]	+= sz;
		}
	}
	push_back( cur );
}

} /* namespace efs */
