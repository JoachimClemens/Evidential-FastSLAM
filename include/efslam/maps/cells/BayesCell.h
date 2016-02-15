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

#ifndef EFS_BAYES_CELL_H_
#define EFS_BAYES_CELL_H_

#include "PointAccumulator.h"

namespace efs {


template<int N>
class BayesCell : public PointAccumulator<N> {
public:
	using typename PointAccumulator<N>::PointNw;

	inline					BayesCell( int i = 0 );
	inline					BayesCell( const std::string &str );
	inline virtual			~BayesCell();

	inline void				updateNoHit();
	inline void				updateHit( const PointNw &p );
	inline void				integrate( const BayesCell &c );

	inline double			fullness() const;
	inline double			entropy() const;

	inline bool				operator==( const BayesCell &other ) const noexcept;

	inline bool				pruneEqual( const BayesCell &other ) const noexcept;

	inline void				fromStr( const std::string &str );
	inline std::string		toStr() const;

	virtual inline size_t	bytes() const noexcept;

	virtual	inline 	constexpr	CellBase::TypeE	type() const	{ return CellBase::TypeE::CELL_BAYES; }
	static	inline	constexpr 	CellBase::TypeE	staticType()	{ return CellBase::TypeE::CELL_BAYES; }

    friend std::ostream& operator<<( std::ostream& ostr, const BayesCell* c ){
        return operator<<( ostr,(*c) );
    }

    friend std::ostream& operator<<( std::ostream& ostr, const BayesCell& c ){
    	if( c.m_visits )
    		ostr << c.fullness();
    	else
    		ostr << "0.5";
        return ostr;
    }

protected:
    using Acc = typename PointAccumulator<N>::Acc;

	uint32_t	m_visits;

public:
	GETTER( visits );
};


} /* namespace efs */

#include "BayesCell.hpp"

#endif /* EFS_BAYES_CELL_H_ */
