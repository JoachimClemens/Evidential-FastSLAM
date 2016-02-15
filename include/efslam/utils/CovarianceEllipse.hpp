/*
 * Software License Agreement (BSD License)
 *
 *  Evidential FastSLAM - An evidential approach to SLAM
 *  Copyright (c) 2013-2016, Joachim Clemens, Thomas Reineking, Tobias Kluth
 *  All rights reserved.
 *
 *  This file is partially based on the ROS CovarianceEllipsoid class
 *  Copyright (c) 2009, Daniel Stonier
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

CovarianceEllipse::CovarianceEllipse() :
	m_axes( Matrix2w::Identity() ),
	m_lengths( Point2w::Ones() )
{

}


CovarianceEllipse::CovarianceEllipse( const Matrix2w &cov ) {
	compute( cov );
}


void
CovarianceEllipse::compute( const Matrix2w &cov ) {
	// Based on ROS' ecl::CovarianceEllipsoid2f

	// Eigenvalues
	float 	a = cov( 0, 0 ),
			b = cov( 0, 1 ),
			c = cov( 1, 0 ),
			d = cov( 1, 1 );

	float tmp = sqrtf( (a+d)*(a+d) / 4 - a*d + b*c );
    m_lengths << sqrtf( (a+d)/2 + tmp ), sqrtf( (a+d)/2 - tmp );


	// Eigenvectors
    if( c != 0 ) {
        m_axes( 0, 0 ) = m_lengths( 0 )*m_lengths( 0 ) - d;
        m_axes( 1, 0 ) = c;
        m_axes( 0, 1 ) = m_lengths( 1 )*m_lengths( 1 ) - d;
        m_axes( 1, 1 ) = c;
    } else if( b != 0 ) {
    	m_axes( 0, 0 ) = b;
    	m_axes( 1, 0 ) = m_lengths( 0 )*m_lengths( 0 ) - a;
    	m_axes( 0, 1 ) = b;
    	m_axes( 1, 1 ) = m_lengths( 1 )*m_lengths( 1 ) - a;
    } else {
    	if( a > d ) {
    		m_axes <<	1, 0,
						0, 1;
    	} else {
    		m_axes <<	0, 1,
						1, 0;
    	}
    }

	// Normalize Evectors
    m_axes.block<2,1>( 0, 0 ).normalize();
    m_axes.block<2,1>( 0, 1 ).normalize();
}


float
CovarianceEllipse::rotation() const {
	return atan2f( (float) m_axes( 1, 0 ), (float) m_axes( 0, 0 ) );
}


} /* namespace efs */

