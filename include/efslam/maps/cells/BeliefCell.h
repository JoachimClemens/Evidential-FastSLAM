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

#ifndef EFS_BELIEF_CELL_H_
#define EFS_BELIEF_CELL_H_

#include "BayesCell.h"

#include "efslam/utils/BeliefFunction.h"

namespace efs {


template<int N>
class BeliefCell : public BayesCell<N> {
public:
	using typename BayesCell<N>::PointNw;

	inline					BeliefCell( int i = 0 );
	inline					BeliefCell( const BeliefFunction &bf );
	inline					BeliefCell( const std::string &str );
	inline virtual			~BeliefCell();

	inline void				updateNoHit( const BeliefFunction &bf, bool behind = false );
	inline void				updateHit( const BeliefFunction &bf, const PointNw &p = PointNw::Zero() );
	inline void				integrate( const BeliefCell &c );

	inline double			entropy() const;

	inline bool				operator==( const BeliefCell &other ) const noexcept;

	inline bool				prune() const noexcept;
	inline bool				pruneEqual( const BeliefCell &other ) const noexcept;

	virtual inline size_t	bytes() const noexcept;

	inline void				fromStr( const std::string &str );
	inline std::string		toStr() const;

	virtual	inline 	constexpr CellBase::TypeE	type() const	{ return CellBase::TypeE::CELL_BELIEF; }
	static	inline	constexpr CellBase::TypeE	staticType()	{ return CellBase::TypeE::CELL_BELIEF; }

    friend std::ostream& operator<<( std::ostream& ostr, const BeliefCell* c ){
        return operator<<( ostr,(*c) );
    }

    friend std::ostream& operator<<( std::ostream& ostr, const BeliefCell& c ){
        ostr << c.m_mass;
        return ostr;
    }

protected:
    using Acc = typename PointAccumulator<N>::Acc;

	BeliefFunction	m_mass;

public:
	GETTER( mass );
	SETTER( mass );
};


} /* namespace efs */

#include "BeliefCell.hpp"

#endif /* EFS_BELIEF_CELL_H_ */
