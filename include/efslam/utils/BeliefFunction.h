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

#ifndef EFS_BELIEFFUNCTION_H_
#define EFS_BELIEFFUNCTION_H_

namespace efs {

struct BeliefFunction {
	inline					BeliefFunction();
	inline					BeliefFunction( float Theta, float o, float e );

	inline float&			operator[]( size_t i ) 			{ return m[i]; };
	inline const float&		operator[]( size_t i ) const	{ return m[i]; };

	inline float			emptyset() const;

	inline float			normalize();
	inline BeliefFunction	normalized() const;

	inline void				conjunctive( const BeliefFunction &other );
	inline void				dempster( const BeliefFunction &other );

	inline float			pignistic() const;
	inline float			internalConflict() const;

	inline void				set( float Theta, float o, float e );

	inline bool				operator==( const BeliefFunction &other ) const noexcept;
	inline bool				operator!=( const BeliefFunction &other ) const noexcept;

	// TODO: Implement other uncertainty measures

	union {
		float		m[3];
		struct {
			float	Theta,
					o,
					e;
		};
	};

    friend std::ostream&    operator<<( std::ostream& ostr, const BeliefFunction* b ) {
        return operator<<( ostr, *b );
    }
    friend std::ostream&    operator<<( std::ostream& ostr, const BeliefFunction& b ) {
    	ostr << "T: " << b.Theta << "  o: " << b.o << "  e: " << b.e;
    	return ostr;
    }

};

} /* namespace efs */

#include "BeliefFunction.hpp"

#endif /* EFS_BELIEFFUNCTION_H_ */
