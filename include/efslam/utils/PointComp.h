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

#ifndef EFS_POINTCOMP_H_
#define EFS_POINTCOMP_H_

#include "Point.h"

namespace efs {

// general version
template<typename Scalar, int N>
struct PointComperator {
	bool operator()( const Point<Scalar, N>& a, const Point<Scalar, N>& b ) const {
		for( int i = 0; i < N; i++ ) {
			if( a[i] < b[i] )
				return true;
			else if(  a[i] > b[i] )
				return false;
		}
		return false;
	}
};

// partially specialized for points of size 2
template<typename Scalar>
struct PointComperator<Scalar, 2> {
	bool operator()( const Point<Scalar, 2>& a, const Point<Scalar, 2>& b ) const {
		return (a[0] < b[0]) || ( (a[0] == b[0]) && (a[1] < b[1]) );
	}
};

// partially specialized for points of size 3
template<typename Scalar>
struct PointComperator<Scalar, 3> {
	bool operator()( const Point<Scalar, 3>& a, const Point<Scalar, 3>& b ) const {
		return (a[0] < b[0]) || ( (a[0] == b[0]) && (a[1] < b[1]) ) || ( (a[0] == b[0]) && (a[1] == b[1]) && (a[2] < b[2]) );
	}
};


using PointComperator2i		= PointComperator<int, 2>;
using PointComperator2d		= PointComperator<double, 2>;
using PointComperator2f		= PointComperator<float, 2>;
using PointComperator2w		= PointComperator<world_t, 2>;
using PointComperator2m		= PointComperator<map_t, 2>;

using PointComperator3i		= PointComperator<int, 3>;
using PointComperator3d		= PointComperator<double, 3>;
using PointComperator3f		= PointComperator<float, 3>;
using PointComperator3w		= PointComperator<world_t, 3>;
using PointComperator3m		= PointComperator<map_t, 3>;

} /* namespace efs */

#endif /* EFS_POINTCOMP_H_ */
