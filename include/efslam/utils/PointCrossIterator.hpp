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

template<int N>
PointCrossIterator<N>::PointCrossIterator( const Pointm<N> &start, const Pointm<N> &end ) :
	m_start( start ),
	m_end( end ),
	m_inc( Pointm<N>::Ones() ),
	m_endReached( start == end ),
	m_idx( N-1 ),
	m_midIdx( round( m_idx / 2.0 ) )
{
	for( int i = 0; i < N; i++ )
		m_mid[i] = m_start[i] + round( (m_end[i] - m_start[i] - 1) / 2.0 );	// -1 is because m_end is behind the range
	m_cur 	 	= m_mid;
	m_cur[N-1] 	= m_start[N-1];
}


template<int N>
PointCrossIterator<N>::PointCrossIterator( const Pointm<N> &start, const Pointm<N> &end, const Pointm<N> &inc ) :
	m_start( start ),
	m_end( end ),
	m_inc( inc ),
	m_endReached( start == end ),
	m_idx( N-1 ),
	m_midIdx( round( m_idx / 2.0 ) )
{
	for( int i = 0; i < N; i++ )
		m_mid[i] = m_start[i] + round( (m_end[i] - m_start[i] - 1) / 2.0 );	// -1 is because m_end is behind the range
	m_cur 	 	= m_mid;
	m_cur[N-1]	= m_start[N-1];
}


template<int N>
void
PointCrossIterator<N>::operator++( int ) {
	++(*this);
}


template<int N>
void
PointCrossIterator<N>::operator++() {
	if( m_endReached )
		return;

	m_cur[m_idx] += m_inc[m_idx];

	// consider the center (m_cur == m_mid) only once
	if( m_idx != m_midIdx && m_cur[m_idx] == m_mid[m_idx] )
		m_cur[m_idx] += m_inc[m_idx];

	// end for this idx reached?
	if( m_cur[m_idx] >= m_end[m_idx] ) {
		m_cur[m_idx] = m_mid[m_idx];
		m_idx--;
		if( m_idx >= 0 ) {
			m_cur[m_idx] = m_start[m_idx];
		} else {
			m_endReached = true;
		}
	}
}

} /* namespace efs */
