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

#ifndef EFS_CELLCOLOR_H_
#define EFS_CELLCOLOR_H_

#include "Color.h"

#include "efslam/maps/cells/BayesCell.h"
#include "efslam/maps/cells/BeliefCell.h"


namespace efs {

enum ColorModeE {
	_COLOR_MODE_MIN_ = -1,
	COLOR_MODE_DEFAULT,
	COLOR_MODE_ALPHA_EMPTY,
	COLOR_MODE_ALPHA_THETA,
	COLOR_MODE_ALPHA_NOT_OCCUPIED,
	_COLOR_MODE_MAX_
};


template<typename CellType>
class CellColor {
};


template<int N>
class CellColor< BayesCell<N> > {
public:
	static inline		 	Color	getColor( const BayesCell<N> &cell, bool normalize = false, uint8_t *alpha = nullptr, ColorModeE = COLOR_MODE_DEFAULT, double alphaScale = 1.0 );
	static inline constexpr Color	defaultColor();
};


template<int N>
class CellColor< BeliefCell<N> > {
public:
	static inline		 	Color	getColor( const BeliefCell<N> &cell, bool normalize = false, uint8_t *alpha = nullptr, ColorModeE = COLOR_MODE_DEFAULT, double alphaScale = 1.0 );
	static inline constexpr Color	defaultColor();
};

} /* namespace efs */

#include "CellColor.hpp"

#endif /* EFS_CELLCOLOR_H_ */
