/*
 * Software License Agreement (BSD License)
 *
 *  Evidential FastSLAM - An evidential approach to SLAM
 *  Copyright (c) 2013-2016, Joachim Clemens, Thomas Reineking, Tobias Kluth
 *  All rights reserved.
 *
 *  This file is partially based on the GMapping PointAccumulator class
 *  Copyright (c) 2004-2007, Giorgio Grisetti, Cyrill Stachniss, Wolfram Burgard
 *  Originally licensed under the Creative Commons (Attribution-NonCommercial-ShareAlike).
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

#ifndef EFS_POINT_ACCUMULATOR_H_
#define EFS_POINT_ACCUMULATOR_H_

#include <memory>

#include "CellBase.h"

#include "efslam/utils/Point.h"
#include "efslam/utils/Convenience.h"

namespace efs {


template<int N>
class PointAccumulator : public CellBase {
public:
	static constexpr int	Dimension	= N;
	using					PointNw 	= Pointw<N>;

	inline						PointAccumulator( int i = 0 );
	inline						PointAccumulator( const PointAccumulator &p );
	inline						PointAccumulator( PointAccumulator &&p );
	inline 	virtual				~PointAccumulator();

	inline 	PointAccumulator&	operator=( const PointAccumulator &p );
	inline	PointAccumulator&	operator=( PointAccumulator &&p );

 	inline 	void				update( const PointNw &p );
 	inline 	void				integrate( const PointAccumulator &pa );

	inline 	PointNw				mean() const;
	inline	uint32_t			hits() const;

	inline 	bool				operator==( const PointAccumulator &other ) const noexcept;

	inline 	bool				prune() const noexcept;
	inline 	bool				pruneEqual( const PointAccumulator &other ) const noexcept;

	virtual inline size_t		bytes() const noexcept;

	virtual	inline 	constexpr CellBase::TypeE	type() const	{ return CellBase::TypeE::CELL_POINT_ACCUMULATOR; }
	static	inline	constexpr CellBase::TypeE	staticType()	{ return CellBase::TypeE::CELL_POINT_ACCUMULATOR; }

protected:
	struct Acc {
		inline		Acc() 					: sum( PointNw::Zero() ), hits( 0 ) {};
		inline		Acc( const PointNw &p ) : sum( p ), hits( 1 ) {};
		inline		~Acc() 					{};

		inline Acc& operator+=( const PointNw &p ) noexcept {
			sum += p;
			hits++;
			return *this;
		}

		inline Acc& operator+=( const Acc &a ) noexcept {
			sum 	+= a.sum;
			hits	+= a.hits;
			return *this;
		}

		PointNw		sum;
		uint32_t	hits;
	};

	std::unique_ptr<Acc>	m_acc;
};


} /* namespace efs */


#include "PointAccumulator.hpp"

#endif /* EFS_POINT_ACCUMULATOR_H_ */
