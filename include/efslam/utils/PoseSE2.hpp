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

#ifndef EFS_POSESE2_HPP_
#define EFS_POSESE2_HPP_

#include "PoseBase.hpp"
#include "Convenience.h"


namespace efs {

PoseSE2::PoseSE2() :
	m_translation( Point2w::Zero() ),
	m_rotation( 0 )
{
	// Nothing else to do here
}


PoseSE2::PoseSE2( PoseSE2 &&other ) :
	m_translation( std::move( other.m_translation ) ),
	m_rotation( std::move( other.m_rotation ) )
{
	// Nothing else to do here
}


PoseSE2::PoseSE2( const PoseSE2 &other ) :
	m_translation( other.m_translation ),
	m_rotation( other.m_rotation )
{
	// Nothing else to do here
}


PoseSE2::PoseSE2( world_t x, world_t y, double phi_ ) :
	m_translation( x, y ),
	m_rotation( phi_ )
{
	// Nothing else to do here
}


PoseSE2::PoseSE2( const Point2w &pos_, double phi_ ) :
	m_translation( pos_ ),
	m_rotation( phi_ )
{
	// Nothing else to do here
}


PoseSE2::PoseSE2( const EigenIsometry2w &other ) :
	m_rotation( 0 )
{
	*this = other;
}


PoseSE2::~PoseSE2() {
	// Nothing to do here
}


PoseSE2&
PoseSE2::operator=( const PoseSE2 &other ) {
	if( this != &other ) {
		m_translation	= other.m_translation;
		m_rotation 		= other.m_rotation;
	}
	return *this;
}


PoseSE2&
PoseSE2::operator=( PoseSE2 &&other ) {
	if( this != &other ) {
		m_translation	= std::move( other.m_translation );
		m_rotation 		= std::move( other.m_rotation );
	}
	return *this;
}


PoseSE2
PoseSE2::operator+( const PoseSE2 &p ) const {
	PoseSE2 res = *this;
	return res += p;
}


PoseSE2&
PoseSE2::operator+=( const PoseSE2 &p ) {
	m_translation	+= p.m_translation;
	phi() 			+= p.phi();
	normalizeRotation();

	return *this;
}


PoseSE2
PoseSE2::operator-( const PoseSE2 &p ) const {
	PoseSE2 res = *this;
	return res -= p;
}


PoseSE2&
PoseSE2::operator-=( const PoseSE2 &p ) {
	m_translation	-= p.m_translation;
	phi() 			-= p.phi();
	normalizeRotation();

	return *this;
}


PoseSE2
PoseSE2::compound( const PoseSE2 &p ) const {
	/* Compounding operator according to
	 * 	 Smith, Randall, Matthew Self, and Peter Cheeseman. Estimating uncertain
	 * 	 spatial relationships in robotics. Autonomous robot vehicles. Springer
	 * 	 New York, 1990. 167-193.
	 * 	 (Sects 3.2.1)
	 * 	and
	 * 	 F. Lu and E. Milios. Globally consistent range scan alignment for
	 * 	 environment mapping. Journal of Autonomous Robots, 4:333–349, 1997.
	 *   (Eq. 17-20)
	 * This is basically equivalent to the concatenation of the corresponding transformations.
	 */

	return *this * p;

	/*
	double	sinPhi	= sin( phi() ),
			cosPhi	= cos( phi() );

	return PoseSE2(  x() + cosPhi*p.x() - sinPhi*p.y(),
					y() + sinPhi*p.x() + cosPhi*p.y(),
					phi()    + p.phi() );
	*/
}


PoseSE2
PoseSE2::compoundInv( const PoseSE2 &p ) const {
	/* Inverse compounding operator according to
	 * 	 Smith, Randall, Matthew Self, and Peter Cheeseman. Estimating uncertain
	 * 	 spatial relationships in robotics. Autonomous robot vehicles. Springer
	 * 	 New York, 1990. 167-193.
	 * 	 (Sects 3.2.2)
	 * 	and
	 * 	 F. Lu and E. Milios. Globally consistent range scan alignment for
	 * 	 environment mapping. Journal of Autonomous Robots, 4:333–349, 1997.
	 * 	 (Eq. 21-24)
	 * This is basically equivalent to the concatenation of the corresponding transformations.
	 */

	return p.inverse() *= *this;

	/*
	PoseSE2 	delta	= *this - p;
	double	sinPhi	= sin( p.phi() ),
			cosPhi	= cos( p.phi() );

	return PoseSE2(  cosPhi*delta.x() + sinPhi*delta.y(),
			       -sinPhi*delta.x() + cosPhi*delta.y(),
			       ANGLE_NORMALIZE( delta.phi() ) );
	 */
}


PoseSE2
PoseSE2::inverse() const {
    PoseSE2 res;

    res.m_rotation		= m_rotation.inverse();
    res.normalizeRotation();
    res.m_translation	= res.m_rotation * (m_translation * -1.);

    return res;
}


PoseSE2
PoseSE2::operator*( const PoseSE2 &p ) const {
	PoseSE2 res( *this );
	return res *= p;
}


PoseSE2&
PoseSE2::operator*=( const PoseSE2 &p ) {
    m_translation		+=	m_rotation * p.m_translation;
    m_rotation.angle()	+= 	p.m_rotation.angle();
    normalizeRotation();

    return *this;
}


template<typename Derived>
inline typename std::enable_if<Derived::IsVectorAtCompileTime && Derived::RowsAtCompileTime==2, Point2w>::type
PoseSE2::operator*( const Eigen::MatrixBase<Derived> &p ) const {
	return m_translation + m_rotation * p;
}


void
PoseSE2::normalizeRotation() {
	ANGLE_NORMALIZE_INPLACE( m_rotation.angle() );
}


PoseSE2
PoseSE2::toPoseSE2() const {
	return *this;
}


PoseSE2::operator PoseBase::EigenIsometry2w() const {
	EigenIsometry2w res( m_rotation );
    res.translation() = m_translation;
    return res;
}


PoseSE2&
PoseSE2::operator=( const EigenIsometry2w &other ) {
	m_translation = other.translation();
	m_rotation.fromRotationMatrix( other.rotation() );

	return *this;
}


PoseSE2::EigenVector
PoseSE2::toVector() const {
	return EigenVector( x(), y(), phi() );
}


} /* namespace efs */

#endif /* EFS_POSESE2_HPP_ */
