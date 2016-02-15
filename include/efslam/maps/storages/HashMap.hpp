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


template<typename Cell, int N>
const Cell HashMap<Cell, N>::sm_emptyValue = Cell( 0 );


template<typename Cell, int N>
HashMap<Cell, N>::HashMap( const Pointm<N> &size ) :
	m_size( size )
{
	// nothing to do here
}


template<typename Cell, int N>
HashMap<Cell, N>::HashMap( const HashMap &other ) :
	m_size( other.m_size ),
	m_map( other.m_map )
{
	// nothing to do here
}


template<typename Cell, int N>
HashMap<Cell, N>::HashMap( HashMap &&other )  :
	m_size( std::move( other.m_size ) ),
	m_map( std::move( other.m_map ) )
{
	// nothing to do here
}


template<typename Cell, int N>
HashMap<Cell, N>::~HashMap() {
	// nothing to do here
}


template<typename Cell, int N>
void
HashMap<Cell, N>::clear() {
	m_map.clear();
}


template<typename Cell, int N>
HashMap<Cell, N>&
HashMap<Cell, N>::operator=( const HashMap &other ) {
	if( this != &other ) {
		m_size 	= other.m_size;
		m_map	= other.m_map;
	}
	return *this;
}


template<typename Cell, int N>
HashMap<Cell, N>&
HashMap<Cell, N>::operator=( HashMap &&other ) {
	if( this != &other ) {
		m_size 	= std::move( other.m_size );
		m_map	= std::move( other.m_map );
	}
	return *this;
}


template<typename Cell, int N>
const Cell&
HashMap<Cell, N>::cell( const Pointm<N>& p ) const {
	const auto &c = m_map.find( p );

	if( c != m_map.end() )
		return c->second;
	else
		return sm_emptyValue;
}


template<typename Cell, int N>
Cell&
HashMap<Cell, N>::cell( const Pointm<N>& p ) {
	return m_map[p];
}


template<typename Cell, int N>
template<typename CellOut>
typename HashMap<Cell, N>::template CellList<CellOut>
HashMap<Cell, N>::cells( const Pointm<N> &offset ) const {
	CellList<CellOut> res;

	for( const auto &c : m_map )
		res.push_back( std::pair<Point3m, const CellOut &>( offset + c.first, c.second ) );

	return res;
}


template<typename Cell, int N>
size_t
HashMap<Cell, N>::bytes() const {
	return sizeof( *this ) + sizeof( Cell )*m_map.size();
}


template<typename Cell, int N>
AccessibilityState
HashMap<Cell, N>::cellState( const Pointm<N>& p ) const {
	if( isInside( p ) ) {
		if( isAllocated( p ) )
			return (AccessibilityState) ((int) AS_INSIDE | (int) AS_ALLOCATED);
		else
			return AS_INSIDE;
	} else {
		return AS_OUTSIDE;
	}
}


template<typename Cell, int N>
bool
HashMap<Cell, N>::isInside( const Pointm<N>& p ) const {
	for( size_t i = 0; i < N; i++ )
		if( p[i] < 0 || p[i] >= m_size[i] )
			return false;
	return true;
}


template<typename Cell, int N>
bool
HashMap<Cell, N>::isAllocated( const Pointm<N> &p ) const {
	return m_map.find( p ) != m_map.end();
}


} /* namespace efs */

