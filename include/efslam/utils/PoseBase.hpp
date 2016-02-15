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

#ifndef EFS_POSEBASE_HPP_
#define EFS_POSEBASE_HPP_

#include "PoseSE2.h"


namespace efs {

// Declaration of specialized methods
template<>
inline Pointw<2>
PoseBase::pos<2>() const;


template<int N>
typename PoseSE<N>::Type
PoseBase::toPoseSE() const {
	static_assert( N == 2, "N must be 2." );
}


template<>
inline typename PoseSE<2>::Type
PoseBase::toPoseSE<2>() const {
	return this->toPoseSE2();
}


PoseBase::operator PoseSE2() const {
	return this->toPoseSE2();
}


PoseBase::operator Eigen::Isometry2d() const {
	return EigenIsometry2w( *this ).cast<double>();
}


template<int N>
Pointw<N>
PoseBase::pos() const {
	static_assert( N == 2, "N must be 2." );
}


template<>
inline Pointw<2>
PoseBase::pos<2>() const {
	return this->pos2D();
}


} /* namespace efs */

#endif /* EFS_POSEBASE_HPP_ */
