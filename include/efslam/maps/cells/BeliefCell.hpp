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
BeliefCell<N>::BeliefCell( int i ) :
	BayesCell<N>( i )
{
	// Nothing else to do here
}


template<int N>
BeliefCell<N>::BeliefCell( const BeliefFunction &bf ) :
	BayesCell<N>(),
	m_mass( bf )
{
	// Nothing else to do here
}


template<int N>
BeliefCell<N>::BeliefCell( const std::string &str ) {
	fromStr( str );
}


template<int N>
BeliefCell<N>::~BeliefCell() {
	// Nothing to do here
}


template<int N>
void
BeliefCell<N>::updateNoHit( const BeliefFunction &bf, bool behind ) {
	if( !behind ) // do not update baysian part of belief cell, if cell is behind measurement. Otherwise it would be treaded as empty.
		BayesCell<N>::updateNoHit();
	m_mass.dempster( bf );
}


template<int N>
void
BeliefCell<N>::updateHit( const BeliefFunction &bf, const PointNw &p ) {
	BayesCell<N>::updateHit( p );
	m_mass.dempster( bf );
}


template<int N>
void
BeliefCell<N>::integrate( const BeliefCell<N> &c ) {
	BayesCell<N>::integrate( c );
	m_mass.conjunctive( c.m_mass );
}


template<int N>
double
BeliefCell<N>::entropy() const {
	return m_mass.internalConflict();
}


template<int N>
bool
BeliefCell<N>::operator==( const BeliefCell<N> &other ) const noexcept {
	return BayesCell<N>::operator==( other ) && m_mass == other.m_mass ;
}


template<int N>
bool
BeliefCell<N>::prune() const noexcept {
	return m_mass.o == 0 && BayesCell<N>::prune();
}


template<int N>
bool
BeliefCell<N>::pruneEqual( const BeliefCell<N> &other ) const noexcept {
	return m_mass.e == other.m_mass.e && m_mass.o == other.m_mass.o && BayesCell<N>::pruneEqual( other );
}


template<int N>
size_t
BeliefCell<N>::bytes() const noexcept {
	return sizeof( BeliefCell<N> ) + (this->m_acc ? sizeof( Acc ) : 0);
}


template<int N>
void
BeliefCell<N>::fromStr( const std::string &str ) {
	auto splitted = split( str, ' ' );

	if( splitted.size() != N + 2 + 3 )
		throw std::runtime_error( to_string( "Wrong number of values, expected " ) + to_string( N + 2 + 3 ) + ", got " + to_string( splitted.size() ) );

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

	this->m_visits = std::stol( splitted[N+1] );

	for( int i = 0; i < 3; i++ )
		m_mass.m[i]	= std::stod( splitted[N+2+i] );
}


template<int N>
std::string
BeliefCell<N>::toStr() const {
	std::ostringstream	sstr;

	sstr << std::setprecision( 30 );

	if( this->m_acc )
		sstr << this->m_acc->sum.transpose() << " " << this->m_acc->hits << " ";
	else
		for( int i = 0; i < N + 1; i++ )
			sstr << "0 ";

	sstr << this->m_visits;

	for( int i = 0; i < 3; i++ )
		sstr << " " << m_mass.m[i];

	return sstr.str();
}

} /* namespace efs */
