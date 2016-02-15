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


/*********************************
 * BayesCell
 *********************************/

template<int N>
Color
CellColor< BayesCell<N> >::getColor( const BayesCell<N> &cell, bool normalize, uint8_t *alpha, ColorModeE colorMode, double alphaScale ) {
	double v = cell.fullness();

	if( v < 0 ) {
		if( alpha )
			*alpha = 0;
		return defaultColor();
	}

	if( alpha ) {
		switch( colorMode ) {
			case COLOR_MODE_ALPHA_EMPTY:
				*alpha = MAX( 0.0, (alphaScale - 1.0+v) ) * 255;
				break;

			case COLOR_MODE_ALPHA_NOT_OCCUPIED:
				*alpha = v * 255;
				break;

			case COLOR_MODE_ALPHA_THETA:
			case COLOR_MODE_DEFAULT:
			default:
				*alpha = 255;
				break;
		}
	}

	uint8_t grayVal = (uint8_t) (255 - (uint8_t) (255 * v));
	return { grayVal, grayVal, grayVal };
}


template<int N>
constexpr Color
CellColor< BayesCell<N> >::defaultColor() {
	return { 200, 200, 255 };
}


/*********************************
 * BeliefCell
 *********************************/

template<int N>
Color
CellColor< BeliefCell<N> >::getColor( const BeliefCell<N> &cell, bool normalize, uint8_t *alpha, ColorModeE colorMode, double alphaScale ) {
	const BeliefFunction	*m;
	BeliefFunction			normalized;

	if( !normalize ) {
		m = &cell.mass();
	} else {
		normalized = cell.mass().normalized();
		m = &normalized;
	}

	if( alpha ) {
		switch( colorMode ) {
			case COLOR_MODE_ALPHA_THETA:
				*alpha = (alphaScale - m->Theta) * 255;  		// 1 - Theta -> Alpha channel
				break;

			case COLOR_MODE_ALPHA_EMPTY:
				*alpha = MAX( 0.0, (alphaScale - m->e) ) * 255;	// 1 - Empty -> Alpha channel
				break;

			case COLOR_MODE_ALPHA_NOT_OCCUPIED:
				*alpha = m->o * 255;		 					// o -> Alpha channel
				break;

			case COLOR_MODE_DEFAULT:
			default:
				*alpha = 255;            						// Full Alpha (Emptyset will be black)
				break;
		}
	}

	return { 	(uint8_t)(m->o		* 255),		// Occupied -> Red
				(uint8_t)(m->e		* 255),     // Empty -> Green
				(uint8_t)(m->Theta	* 255)  };	// Theta -> Blue
}


template<int N>
constexpr Color
CellColor< BeliefCell<N> >::defaultColor() {
	return { 0, 0, 255 };
}

} /* namespace efs */

