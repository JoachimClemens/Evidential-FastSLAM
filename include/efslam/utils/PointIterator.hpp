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
PointIterator<N>::PointIterator( const Pointm<N> &end ) :
	m_start( Pointm<N>::Zero() ),
	m_cur( m_start ),
	m_end( end ),
	m_inc( Pointm<N>::Ones() ),
	m_endReached( m_start == end )
{
}


template<int N>
PointIterator<N>::PointIterator( const Pointm<N> &start, const Pointm<N> &end ) :
	m_start( start ),
	m_cur( start ),
	m_end( end ),
	m_inc( Pointm<N>::Ones() ),
	m_endReached( start == end )
{
}


template<int N>
PointIterator<N>::PointIterator( const Pointm<N> &start, const Pointm<N> &end, const Pointm<N> &inc ) :
	m_start( start ),
	m_cur( start ),
	m_end( end ),
	m_inc( inc ),
	m_endReached( start == end )
{
}


template<int N>
void
PointIterator<N>::operator++( int ) {
	++(*this);
}


template<int N>
void
PointIterator<N>::operator++() {
	if( m_endReached )
		return;

	for( int i = N-1; i >= 0; i-- ) {
		m_cur[i] += m_inc[i];
		if( m_cur[i] < m_end[i] ) {
			break;
		} else {
			if( i == 0 )
				m_endReached = true;
			m_cur[i] = m_start[i];
		}
	}
}

} /* namespace efs */
