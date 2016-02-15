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

#ifndef EFS_POINT_H_
#define EFS_POINT_H_

#include <Eigen/Dense>

/*
#ifdef HAVE_PCL
#	include<pcl/point_types.h>
#endif
*/

namespace efs {

using world_t	= float;
using map_t		= int32_t;	// TODO: Use int16_t instead?


template<typename Scalar, int N> using Point	= Eigen::Matrix<Scalar, N, 1>;

/*
template<int N> 			using Pointi	= Point<int32_t, N>;
template<int N> 			using Pointd	= Point<double, N>;
template<int N> 			using Pointf	= Point<float, N>;
*/

template<int N> 			using Pointw	= Point<world_t, N>;
template<int N> 			using Pointm	= Point<map_t, N>;

template<typename Scalar> 	using Point2	= Point<Scalar, 2>;
template<typename Scalar> 	using Point3	= Point<Scalar, 3>;


/*
using	Point2i = Point<int32_t, 2>;
using	Point2d = Point<double, 2>;
using	Point2f = Point<float, 2>;
*/

using	Point2w = Point<world_t, 2>;
using	Point2m = Point<map_t, 2>;


/*
using	Point3i = Point<int32_t, 3>;
using	Point3d = Point<double, 3>;
using	Point3f = Point<float, 3>;
*/

using	Point3w = Point<world_t, 3>;
using	Point3m = Point<map_t, 3>;


/*
template<int N>
inline Pointw<N>
toPointw( const Pointw<N> &p ) {
	return p;
}

template<int N>
inline Eigen::Map< Pointw<N> >
getMap( Pointw<N> &p ) {
	return Eigen::Map< Pointw<N> >( p );
}


#ifdef HAVE_PCL
template<int N>
inline Pointw<N>
toPointw( const pcl::PointXYZ &p ) {
	// Probably does not work with world_t=double
	return Eigen::Map< Eigen::Matrix<float, N, 1> >( p.data );
}

template<int N>
inline Eigen::Map< Pointw<N> >
getMap( pcl::PointXYZ &p ) {
	// Probably does not work with world_t=double
	return Eigen::Map< Eigen::Matrix<float, N, 1> >( p.data );
}
#endif
*/


} /* namespace efs */


namespace std {

template <>
struct hash<efs::Point2m> {
	inline size_t operator()( const efs::Point2m & p ) const {
		// According to OctoMap, OcTreeKey: https://github.com/OctoMap/octomap/blob/devel/octomap/include/octomap/OcTreeKey.h
		//return p[0] + 1337*p[1];

		// According to Teschner et al. 2003, "Optimized Spatial Hashing for Collision Detection of Deformable Objects", http://www.beosil.com/download/CollisionDetectionHashing_VMV03.pdf
		return (p[0] * 73856093) ^ (p[1] * 19349663);
	}
};


template <>
struct hash<efs::Point3m> {
	inline size_t operator()( const efs::Point3m & p ) const {
		// According to OctoMap, OcTreeKey: https://github.com/OctoMap/octomap/blob/devel/octomap/include/octomap/OcTreeKey.h
		//return p[0] + 1337*p[1] + 345637*p[2];

		// According to Teschner et al. 2003, "Optimized Spatial Hashing for Collision Detection of Deformable Objects", http://www.beosil.com/download/CollisionDetectionHashing_VMV03.pdf
		return (p[0] * 73856093) ^ (p[1] * 19349663) ^ (p[2] * 83492791);
	}
};

} /* namespace std */


namespace boost {

template<typename T>
class hash;

template<>
struct hash<efs::Point2m> : public std::hash<efs::Point2m> {
};


template<>
struct hash<efs::Point3m> : public std::hash<efs::Point3m> {
};

} /* namespace boost */



#endif /* EFS_POINT_H_ */
