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

#ifndef EFS_POSESE2_H_
#define EFS_POSESE2_H_

#define NO_HPP
#include "PoseBase.h"
#undef NO_HPP

#include "Point.h"
#include "Convenience.h"

namespace efs {

class PoseSE2 : public PoseBase {
public:
	static constexpr int 	Dimension 	= 2;
	static constexpr int 	DOF			= 3;
	static constexpr int	VectorSize	= 3;

	using	EigenVector		= Eigen::Matrix<world_t, VectorSize, 1>;
	using	Rotation		= Eigen::Rotation2D<world_t>;

			inline					PoseSE2();
			inline					PoseSE2( PoseSE2 &&other );
			inline					PoseSE2( const PoseSE2 &other );
			inline					PoseSE2( world_t x, world_t y, double phi );
			inline					PoseSE2( const Point2w &pos, double phi );
			inline					PoseSE2( const EigenIsometry2w &other );
	virtual inline					~PoseSE2();


			inline	PoseSE2&		operator=( const PoseSE2 &other );
			inline	PoseSE2&		operator=( PoseSE2 &&other );

	// this is simple addition and subtraction, not concatenation!
			inline 	PoseSE2			operator+( const PoseSE2 &p ) const;
			inline 	PoseSE2&		operator+=( const PoseSE2 &p );
			inline 	PoseSE2			operator-( const PoseSE2 &p ) const;
			inline 	PoseSE2&		operator-=( const PoseSE2 &p );

	// Note the different order: getting the transformation a followed by b results from a.compound(b) (in contrast to b*a)
			inline	PoseSE2			compound( const PoseSE2 &p ) const;
			inline	PoseSE2			compoundInv( const PoseSE2 &p ) const;
			inline	PoseSE2			inverse() const;
			inline	void			normalizeRotation();

			inline	PoseSE2			operator*( const PoseSE2 &p ) const;
			inline	PoseSE2&		operator*=( const PoseSE2 &p );
	template<typename Derived>
	inline	typename std::enable_if<Derived::IsVectorAtCompileTime && Derived::RowsAtCompileTime==2, Point2w>::type
									operator*( const Eigen::MatrixBase<Derived> &p ) const;

	virtual inline 	PoseSE2			toPoseSE2() const;

	virtual	inline	operator		EigenIsometry2w() const;
			inline	PoseSE2&		operator=( const EigenIsometry2w &other );

			inline	EigenVector		toVector() const;
	virtual inline	Point2w			pos2D() const	{ return m_translation; }

			inline	const world_t&	x() const		{ return m_translation[0]; 		}
			inline	world_t&		x()				{ return m_translation[0]; 		}
			inline	const world_t&	y() const		{ return m_translation[1]; 		}
			inline	world_t&		y()				{ return m_translation[1]; 		}
			inline	world_t			phi() const		{ return m_rotation.angle();	}
			inline	world_t&		phi()			{ return m_rotation.angle();	}
			inline	const Point2w&	pos() const		{ return m_translation;			}
			inline	Point2w&		pos()			{ return m_translation;			}

	virtual inline	double			posNorm() const { return m_translation.norm(); 	}
	virtual inline	double			angNorm() const { return fabs( ANGLE_NORMALIZE( phi() ) ); 		}

	virtual inline	int				dimension() const noexcept	{ return Dimension; }

    friend std::ostream& operator<<( std::ostream& ostr, const PoseSE2* p ){
        return operator<<( ostr, (*p) );
    }

    friend std::ostream& operator<<( std::ostream& ostr, const PoseSE2& p ){
    	ostr << p.m_translation.transpose() << " " << RAD2DEG( p.phi() );
        return ostr;
    }

protected:
	Point2w		m_translation;
	Rotation	m_rotation;

public:
	GETTER( translation );
	SETTER( translation );
	GETTER( rotation );
	SETTER( rotation );
};

} /* namespace efs */

#include "PoseSE2.hpp"


#endif /* EFS_POSESE2_H_ */
