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

#include <assert.h>

namespace efs {

Color
Color::hsv2rgb( double h, double s, double v ) {
	assert( h >= 0 && h < 360 && s >= 0 && s <= 1.0 && v >= 0 && v <= 1.0 );

	double 	c = v * s,
			x = c * ( 1 - fabs( fmod( h / 60.0, 2  ) - 1 ) ),
			m = v - c,
			r = 0,
			g = 0,
			b = 0;

	if( h < 60 ) {
		r = c;
		g = x;
	} else if( h < 120 ) {
		r = x;
		g = c;
	} else if( h < 180 ) {
		g = c;
		b = x;
	} else if( h < 240 ) {
		g = x;
		b = c;
	} else if( h < 300 ) {
		r = x;
		b = c;
	} else if( h < 360 ) {
		r = c;
		b = x;
	} else {
		throw std::invalid_argument( "H larger than 360" );
	}

	return { (uint8_t)( (r + m) * 255 ), (uint8_t)((g + m) * 255), (uint8_t)( (b + m) * 255 ) };
}

} /* namespace efs */
