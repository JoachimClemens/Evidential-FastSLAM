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

// generic methods

template<int N>
inline size_t
HierarchicalStorageHelper<N>::patchIndex( const Pointm<N> &p, const Pointm<N> &size, int magnitude ) {
	size_t 	sum = 0,
			prod;

	for( size_t i = 0; i < N; i++ ) {
		prod = p[i] >> magnitude;
		for( size_t j = i+1; j < N; j++ )
			prod *= size[j];
		sum += prod;
	}

	return sum;
}


template<int N>
inline Pointm<N>
HierarchicalStorageHelper<N>::cellIndex( const Pointm<N> &p, int magnitude ) {
	Pointm<N> res;

	for( size_t i = 0; i < N; i++ )
		res[i] = p[i] - ((p[i] >> magnitude) << magnitude);

	return res;
}


// specialization for N=2

template<>
inline size_t
HierarchicalStorageHelper<2>::patchIndex( const Point2m &p, const Point2m &size, int magnitude ) {
	// x*size_y + y
	return (p[0] >> magnitude)*size[1] + (p[1] >> magnitude);
}


template<>
inline Point2m
HierarchicalStorageHelper<2>::cellIndex( const Point2m &p, int magnitude ) {
	return Point2m( p[0] - ((p[0] >> magnitude) << magnitude), p[1] - ((p[1] >> magnitude) << magnitude) );
}


// specialization for N=3

template<>
inline size_t
HierarchicalStorageHelper<3>::patchIndex( const Point3m &p, const Point3m &size, int magnitude ) {
	// x*size_y*size_z + y*size_z + z
	return (p[0] >> magnitude)*size[1]*size[2] + (p[1] >> magnitude)*size[2] + (p[2] >> magnitude);
}


template<>
inline Point3m
HierarchicalStorageHelper<3>::cellIndex( const Point3m &p, int magnitude ) {
	return Point3m( p[0] - ((p[0] >> magnitude) << magnitude), p[1] - ((p[1] >> magnitude) << magnitude), p[2] - ((p[2] >> magnitude) << magnitude)  );
}

} /* namespace efs */
