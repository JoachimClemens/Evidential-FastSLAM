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

#include <cmath>
#include <sstream>
#include <iomanip>


namespace efs {

template<int N>
BayesCell<N>::BayesCell( int i ) :
	PointAccumulator<N>( i ),
	m_visits( 0 )
{
	// nothing else to do here
}


template<int N>
BayesCell<N>::BayesCell( const std::string &str ) {
	fromStr( str );
}


template<int N>
BayesCell<N>::~BayesCell() {
	// Nothing to do here
}


template<int N>
void
BayesCell<N>::updateNoHit() {
	m_visits++;
}


template<int N>
void
BayesCell<N>::updateHit( const PointNw &p ) {
	PointAccumulator<N>::update( p );
	m_visits++;
}


template<int N>
void
BayesCell<N>::integrate( const BayesCell<N> &c ) {
	PointAccumulator<N>::integrate( c );
	m_visits += c.m_visits;
}


template<int N>
double
BayesCell<N>::fullness() const {
	if( !m_visits )
		return -1;
	return (double) this->hits() / (double) m_visits;
}


template<int N>
double
BayesCell<N>::entropy() const {
	if( !m_visits )
		return -log2( 0.5 );

	auto hits = this->hits();
	if( hits == 0 || hits == m_visits )
		return 0;

	double x = fullness();
	return -(x * log2( x ) + (1 - x) * log2( 1 - x ));
}


template<int N>
bool
BayesCell<N>::operator==( const BayesCell<N> &other ) const noexcept {
	return m_visits == other.m_visits && PointAccumulator<N>::operator==( other );
}


template<int N>
bool
BayesCell<N>::pruneEqual( const BayesCell<N> &other ) const noexcept {
	return m_visits == other.m_visits && PointAccumulator<N>::pruneEqual( other );
}


template<int N>
size_t
BayesCell<N>::bytes() const noexcept {
	return sizeof( BayesCell<N> ) + (this->m_acc ? sizeof( Acc ) : 0);
}


template<int N>
void
BayesCell<N>::fromStr( const std::string &str ) {
	auto splitted = split( str, ' ' );

	if( splitted.size() != N + 2 )
		throw std::runtime_error( to_string( "Wrong number of values, expected " ) + to_string( N + 2 ) + ", got " + to_string( splitted.size() ) );

	uint32_t hits = std::stol( splitted[N] );
	if( hits ) {
		if( !this->m_acc )
			this->m_acc = std::unique_ptr<Acc>( new Acc() );

		this->m_acc->hits = hits;
		for( int i = 0; i < N; i++ )
			this->m_acc->sum[i] = std::stod( splitted[i] );
	} else {
		this->m_acc = nullptr;
	}

	m_visits = std::stol( splitted[N+1] );
}


template<int N>
std::string
BayesCell<N>::toStr() const {
	std::ostringstream	sstr;

	sstr << std::setprecision( 30 );

	if( this->m_acc )
		sstr << this->m_acc->sum.transpose() << " " << this->m_acc->hits << " ";
	else
		for( int i = 0; i < N + 1; i++ )
			sstr << "0 ";

	sstr << this->m_visits;

	return sstr.str();
}



} /* namespace efs */
